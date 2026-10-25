import math
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import SetBool, Trigger
from k7_msgs.msg import ControlReference, FleetState, MotionCommand
from k7_msgs.srv import SelectSource
from k7_msgs.action import FollowPath, FollowLeader
from k7_control.core import finite, project, wrap
from k7_control.ros_utils import diagnostic, fields, parameters, permission, stamp_fresh, yaw
from .formation import LeaderTrail


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.p = parameters(self, dict(profile='trajectory', base_frame='base_link',
            odom_frame='odom', fleet_frame='fleet', publish_fleet=False,
            sensor_timeout=0.4, leader_timeout=0.5, manual_timeout=0.25,
            stop_timeout=3.0, standstill_time=0.3, standstill_speed=0.02,
            wheel_limit=0.35, max_path_points=512, max_task_time=300.0,
            formation_alignment_confirmed=False,
            initial_lateral_tolerance=0.15, initial_yaw_tolerance=0.35,
            fleet_x=0.0, fleet_y=0.0, fleet_yaw=0.0, gap_gain=0.5))
        self.mode = 'IDLE'
        self.action_group = ReentrantCallbackGroup()
        self.token = bytes(16)
        self.session = uuid.uuid4().bytes
        self.pose = self.odom = None
        self.odom_time = self.teleop_time = self.leader_time = -math.inf
        self.still_since = None
        self.goal = self.completion = self.grant = self.revoke = None
        self.manual_grant = None
        self.manual_reply = None
        self.grant_pending = False
        self.leader_sub = None
        self.guard_token = bytes(16)
        self.guard_time = -math.inf
        self.started = self.last_reference = self.last_feedback = 0.0
        self.last_fleet_odom = None
        self.pending_path = None
        self.pub = self.create_publisher(ControlReference, 'control/reference', 1)
        self.candidate = (self.create_publisher(MotionCommand, 'control/candidate', 1)
                          if self.p['profile'] == 'manual' else None)
        self.status = self.create_publisher(DiagnosticArray, 'task/status', 1)
        self.fleet = (self.create_publisher(FleetState, 'fleet/state', 1)
                      if self.p['publish_fleet'] else None)
        self.source = self.create_client(SelectSource, 'control/select_source')
        self.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        self.create_subscription(DiagnosticArray, 'control/status', self.guard_cb, 1)
        self.create_service(SetBool, 'task/set_manual', self.manual_cb,
                            callback_group=self.action_group)
        self.create_service(Trigger, 'task/stop', self.stop_cb)
        if self.p['profile'] == 'manual':
            self.create_subscription(TwistStamped, 'teleop/cmd_vel', self.teleop_cb, 1)
        self.actions = [
            ActionServer(self, FollowPath, 'task/follow_path', self.execute,
                         goal_callback=self.path_goal, cancel_callback=self.cancel,
                         handle_accepted_callback=self.accepted, callback_group=self.action_group),
            ActionServer(self, FollowLeader, 'task/follow_leader', self.execute,
                         goal_callback=self.leader_goal, cancel_callback=self.cancel,
                         handle_accepted_callback=self.accepted, callback_group=self.action_group)]
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_status)
        if self.fleet:
            self.create_timer(0.1, self.publish_fleet)

    def reserve(self, expected):
        if (self.goal is not None or self.grant_pending or self.mode != 'IDLE' or self.p['profile'] != expected or
                any(f is not None and not f.done() for f in (self.grant, self.manual_grant, self.revoke)) or
                not self.source.service_is_ready()):
            return False
        self.mode = 'RESERVED'
        return True

    def path_goal(self, request):
        path = request.path
        ok = (path.header.frame_id == self.p['odom_frame'] and
              2 <= len(path.poses) <= self.p['max_path_points'] and
              finite(request.target_speed, request.position_tolerance, request.yaw_tolerance) and
              0 < request.target_speed <= self.p['wheel_limit'] and
              0.03 <= request.position_tolerance <= 0.5 and 0.02 <= request.yaw_tolerance <= 0.5)
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        ok = ok and all(finite(*p) for p in points) and all(
            math.dist(a, b) > 1e-6 for a, b in zip(points, points[1:]))
        if ok:
            q = path.poses[-1].pose.orientation
            ok = finite(q.x, q.y, q.z, q.w) and abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1) < 0.01
            try:
                project(points, *points[0])
            except ValueError:
                ok = False
        return GoalResponse.ACCEPT if ok and self.reserve('trajectory') else GoalResponse.REJECT

    def leader_goal(self, request):
        ok = (self.p['formation_alignment_confirmed'] and request.leader_topic.startswith('/') and
              request.leader_topic != self.resolve_topic_name('fleet/state') and
              finite(request.gap, request.max_speed) and 0.3 <= request.gap <= 5.0 and
              0 < request.max_speed <= self.p['wheel_limit'])
        return GoalResponse.ACCEPT if ok and self.reserve('formation') else GoalResponse.REJECT

    def accepted(self, handle):
        self.goal = handle
        self.completion = Future()
        if self.mode != 'RESERVED':
            handle.execute()
            self.begin_stop(1, 'start canceled')
            return
        self.started = time.monotonic()
        self.last_reference = self.last_feedback = 0.0
        self.mode = 'STARTING'
        self.grant_pending = True
        self.grant = self.source.call_async(SelectSource.Request(source=2))
        self.grant.add_done_callback(lambda f: self.grant_done(f, 'STARTING'))
        if isinstance(handle.request, FollowLeader.Goal):
            self.trail = LeaderTrail()
            self.leader_time = -math.inf
            self.leader_sub = self.create_subscription(FleetState, handle.request.leader_topic,
                                                       self.leader_cb, 1)
        else:
            self.pending_path = handle.request.path
        handle.execute()

    async def execute(self, handle):
        code, reason = await self.completion
        action = FollowLeader if isinstance(handle.request, FollowLeader.Goal) else FollowPath
        result = action.Result(code=code, reason=reason)
        if code == 0:
            handle.succeed()
        elif code == 1 and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        self.goal = self.completion = None
        return result

    def cancel(self, handle):
        if self.goal is handle:
            self.begin_stop(1, 'canceled')
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    async def manual_cb(self, request, response):
        if not request.data:
            if self.mode in ('MANUAL', 'STARTING_MANUAL'):
                self.begin_stop(1, 'manual disabled')
            response.success, response.message = True, 'stop requested'
            return response
        if not self.reserve('manual'):
            response.success, response.message = False, 'busy or wrong launch profile'
            return response
        self.mode = 'STARTING_MANUAL'
        self.started = time.monotonic()
        self.manual_reply = Future()
        self.grant_pending = True
        self.manual_grant = self.source.call_async(SelectSource.Request(source=1))
        self.manual_grant.add_done_callback(lambda f: self.grant_done(f, 'STARTING_MANUAL'))
        response.success, response.message = await self.manual_reply
        return response

    def grant_done(self, future, expected):
        # A timed-out service can still succeed later; revoke before another task.
        self.grant_pending = False
        try:
            result = future.result()
        except Exception as exc:
            if self.mode == expected:
                self.begin_stop(2, 'authority service failed: '+str(exc))
            return
        if self.mode != expected:
            if result.success:
                self.revoke = self.source.call_async(SelectSource.Request(source=0))
            return
        if not result.success:
            self.begin_stop(2, result.reason)
            return
        self.token = bytes(result.permission_id.uuid)
        if expected == 'STARTING_MANUAL':
            self.mode, self.teleop_time = 'MANUAL', time.monotonic()
            self.answer_manual(True, result.reason)
        else:
            self.mode = 'FOLLOW_LEADER' if self.leader_sub else 'FOLLOW_PATH'
        self.publish_status()

    def answer_manual(self, success, reason):
        if self.manual_reply is not None and not self.manual_reply.done():
            self.manual_reply.set_result((success, reason))

    def stop_cb(self, request, response):
        self.begin_stop(1, 'stop requested')
        response.success, response.message = True, 'accepted; observe task/status for standstill'
        return response

    def begin_stop(self, code, reason):
        if self.mode == 'STOPPING':
            return
        self.mode = 'STOPPING'
        self.stop_code, self.stop_reason, self.stop_started = code, reason, time.monotonic()
        self.token = bytes(16)
        self.revoke = self.source.call_async(SelectSource.Request(source=0))
        self.answer_manual(False, reason)
        self.publish_status()

    def odom_cb(self, msg):
        q = msg.pose.pose
        pose = (q.position.x, q.position.y, yaw(q.orientation))
        if (msg.header.frame_id != self.p['odom_frame'] or not finite(*pose,
                msg.twist.twist.linear.x, msg.twist.twist.angular.z) or
                not stamp_fresh(self, msg.header.stamp, self.p['sensor_timeout'])):
            self.still_since = None
            return
        now = time.monotonic()
        if now-self.odom_time > self.p['sensor_timeout']:
            self.still_since = None
        self.pose, self.odom, self.odom_time = pose, msg, now
        stopped = abs(msg.twist.twist.linear.x) < self.p['standstill_speed'] and abs(msg.twist.twist.angular.z) < self.p['standstill_speed']
        self.still_since = (self.still_since if self.still_since is not None else now) if stopped else None

    def guard_cb(self, msg):
        if stamp_fresh(self, msg.header.stamp, 2.5):
            self.guard_token = permission(fields(msg).get('permission', ''))
            self.guard_time = time.monotonic()

    def teleop_cb(self, msg):
        if (self.mode != 'MANUAL' or msg.header.frame_id != self.p['base_frame'] or
                not stamp_fresh(self, msg.header.stamp, self.p['manual_timeout']) or
                not finite(msg.twist.linear.x, msg.twist.angular.z)):
            return
        self.teleop_time = time.monotonic()
        out = MotionCommand(header=msg.header, twist=msg.twist)
        out.permission_id.uuid = list(self.token)
        self.candidate.publish(out)

    def leader_cb(self, msg):
        if self.pose is None or time.monotonic()-self.odom_time > self.p['sensor_timeout']:
            return
        if (not stamp_fresh(self, msg.header.stamp, self.p['leader_timeout']) or
                msg.header.frame_id != self.p['fleet_frame'] or
                not finite(msg.x, msg.y, msg.yaw, msg.linear_speed, msg.angular_speed)):
            return
        if msg.state == FleetState.FAULT:
            self.begin_stop(2, 'leader fault')
            return
        if msg.linear_speed < -0.01:
            self.begin_stop(2, 'reverse leader motion is not supported')
            return
        # Invert this vehicle's odom-to-fleet transform; namespaces do not align poses.
        c, s = math.cos(self.p['fleet_yaw']), math.sin(self.p['fleet_yaw'])
        dx, dy = msg.x-self.p['fleet_x'], msg.y-self.p['fleet_y']
        point = (c*dx+s*dy, -s*dx+c*dy)
        try:
            if not self.trail.points and self.pose:
                self.trail.seed(self.pose, point, wrap(msg.yaw-self.p['fleet_yaw']),
                    bytes(msg.session_id.uuid), self.p['initial_lateral_tolerance'],
                    self.p['initial_yaw_tolerance'])
            self.trail.append(point, bytes(msg.session_id.uuid))
        except ValueError as exc:
            self.begin_stop(2, str(exc))
            return
        self.leader_time, self.leader_speed = time.monotonic(), msg.linear_speed

    def publish_reference(self, path, speed):
        msg = ControlReference(path=path, speed_limit=float(speed))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['odom_frame']
        msg.permission_id.uuid = list(self.token)
        self.pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.mode.startswith('STARTING') and now-self.started > self.p['stop_timeout']:
            self.begin_stop(2, 'authority grant timeout')
        if self.mode in ('FOLLOW_PATH', 'FOLLOW_LEADER', 'MANUAL'):
            if now-self.odom_time > self.p['sensor_timeout']:
                self.begin_stop(2, 'odometry timeout')
            elif now-self.guard_time > 2.5 or (now-self.started > 1.5 and self.guard_token != self.token):
                self.begin_stop(2, 'authority lost')
            elif self.mode == 'MANUAL' and now-self.teleop_time > self.p['manual_timeout']:
                self.begin_stop(2, 'manual command timeout')
        if self.mode == 'FOLLOW_PATH':
            self.path_tick(now)
        elif self.mode == 'FOLLOW_LEADER':
            self.formation_tick(now)
        if self.mode == 'STOPPING':
            try:
                ack = self.revoke.done() and self.revoke.result().success
            except Exception:
                ack = False
            ack = ack and not self.grant_pending
            stopped = (now-self.odom_time <= self.p['sensor_timeout'] and self.still_since is not None
                       and now-self.still_since >= self.p['standstill_time'])
            if ack and stopped:
                self.finish_stop(self.stop_code, self.stop_reason)
            elif now-self.stop_started > self.p['stop_timeout']:
                self.finish_stop(2, 'stop acknowledgement or standstill timeout')

    def path_tick(self, now):
        goal = self.goal.request
        path = [(p.pose.position.x, p.pose.position.y) for p in goal.path.poses]
        _, cte, heading, arc, total = project(path, *self.pose[:2])
        end, prev = path[-1], path[-2]
        tangent = math.atan2(end[1]-prev[1], end[0]-prev[0])
        crossed = ((self.pose[0]-end[0])*math.cos(tangent) +
                   (self.pose[1]-end[1])*math.sin(tangent)) >= 0
        arrived = math.dist(self.pose[:2], end) <= goal.position_tolerance or (crossed and abs(cte) <= goal.position_tolerance)
        heading_error = wrap(yaw(goal.path.poses[-1].pose.orientation)-self.pose[2])
        if arrived:
            ok = abs(heading_error) <= goal.yaw_tolerance
            self.begin_stop(0 if ok else 2, 'path complete' if ok else 'end reached outside yaw tolerance')
            return
        if now-self.started > self.p['max_task_time']:
            self.begin_stop(2, 'path time budget exceeded')
            return
        # Bounded reference lease also prevents a frozen task from driving indefinitely.
        if now-self.last_reference >= 0.2:
            self.publish_reference(goal.path, goal.target_speed)
            self.last_reference = now
        if now-self.last_feedback >= 0.5:
            self.goal.publish_feedback(FollowPath.Feedback(progress=float(arc/max(total, 1e-6)),
                lateral_error=float(cte), heading_error=float(heading_error),
                speed=float(self.odom.twist.twist.linear.x), state=self.mode))
            self.last_feedback = now

    def formation_tick(self, now):
        if now-self.leader_time > self.p['leader_timeout']:
            if now-self.started > self.p['leader_timeout']:
                self.begin_stop(2, 'leader timeout')
            return
        if now-self.last_reference < 0.1:
            return
        goal = self.goal.request
        try:
            points, speed, error = self.trail.reference(self.pose[:2], goal.gap,
                self.leader_speed, goal.max_speed, self.p['gap_gain'])
        except ValueError:
            return
        path = Path()
        path.header.frame_id = self.p['odom_frame']
        for x, y in points:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
            path.poses.append(p)
        self.publish_reference(path, speed)
        self.last_reference = now
        if now-self.last_feedback >= 0.5:
            self.goal.publish_feedback(FollowLeader.Feedback(gap_error=float(error),
                lateral_error=float(project(points, *self.pose[:2])[1]),
                leader_age=float(now-self.leader_time), state=self.mode))
            self.last_feedback = now

    def finish_stop(self, code, reason):
        self.mode = 'FAULT' if code == 2 else 'IDLE'
        if self.leader_sub:
            self.destroy_subscription(self.leader_sub)
            self.leader_sub = None
        if self.completion is not None and not self.completion.done():
            self.completion.set_result((code, reason))
        self.publish_status()

    def publish_status(self):
        self.status.publish(diagnostic(self, 'task_manager',
            dict(state=self.mode, permission=self.token.hex()),
            2 if self.mode == 'FAULT' else 0, getattr(self, 'stop_reason', '')))

    def publish_fleet(self):
        if not self.p['formation_alignment_confirmed']:
            return
        if self.odom is None or self.odom.header.stamp == self.last_fleet_odom:
            return
        if time.monotonic()-self.odom_time > self.p['sensor_timeout']:
            return
        self.last_fleet_odom = self.odom.header.stamp
        c, s = math.cos(self.p['fleet_yaw']), math.sin(self.p['fleet_yaw'])
        msg = FleetState()
        msg.header.stamp, msg.header.frame_id = self.odom.header.stamp, self.p['fleet_frame']
        msg.session_id.uuid = list(self.session)
        msg.x = self.p['fleet_x']+c*self.pose[0]-s*self.pose[1]
        msg.y = self.p['fleet_y']+s*self.pose[0]+c*self.pose[1]
        msg.yaw = wrap(self.pose[2]+self.p['fleet_yaw'])
        msg.linear_speed, msg.angular_speed = self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z
        msg.state = (FleetState.FAULT if self.mode == 'FAULT' else FleetState.STOPPING if self.mode == 'STOPPING'
                     else FleetState.IDLE if self.mode == 'IDLE' else FleetState.RUNNING)
        self.fleet.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
