"""Per-vehicle reference paths for a three-car formation transition.

Each car runs its own FollowPath goal in its own odom frame; the formation is
encoded in the path (initial slot blended into target slot along the shape), so
no cross-vehicle state (FleetState/FollowLeader) is needed.

Add a trajectory by writing a ``shape(count)`` returning ``[(x, y), ...]`` and
registering it in SHAPES; the formation blend is applied automatically.
"""
import math

# Slots are (gap, lateral): gap = arc length behind the leader along its path,
# lateral = offset to the left of its heading (metres). Same convention as
# k7_tasks/formation.py LeaderTrail.slot().
# Initial: a column, leader first, 0.50 m between cars.
INITIAL_FORMATION = {
    'car1': (0.0, 0.0),
    'car2': (0.5, 0.0),
    'car3': (1.0, 0.0),
}
# Target: a triangle with the leader at the apex; both followers 0.9 m back and
# +-0.5 m lateral (1.0 m base).
TARGET_FORMATION = {
    'car1': (0.0, 0.0),
    'car2': (0.9, 0.5),
    'car3': (0.9, -0.5),
}


def spiral(count=500, inner=1.0, outer=2.2, turns=2):
    """Outward spiral starting at (0, 0): radius inner -> outer over `turns`."""
    start_angle = -math.pi / 2.0
    end_angle = turns * 2 * math.pi
    growth = (outer - inner) / end_angle
    points = []
    for i in range(count):
        a = start_angle + end_angle * i / (count - 1)
        r = inner + growth * (a - start_angle)
        points.append((r * math.cos(a), inner + r * math.sin(a)))
    return points


def figure8(count=500, half_length=4.0, lateral=2.0):
    """Lemniscate in an 8 m x 4 m box, starting at (0, 0)."""
    points = []
    for i in range(count):
        t = 2.0 * math.pi * i / (count - 1)
        points.append((half_length * math.sin(t), lateral * math.sin(2.0 * t)))
    return points


SHAPES = {'spiral': spiral, 'figure8': figure8}


def formation_path(vehicle='car1', shape='spiral'):
    """One car's (x, y, yaw) path in its own odom frame.

    The leader follows `shape`; followers ride the same shape shifted by their
    slot, blended smoothly from INITIAL_FORMATION to TARGET_FORMATION along the
    path. Every path starts at (0, 0), so each car is placed at its initial slot
    and runs its own FollowPath goal.
    """
    if vehicle not in INITIAL_FORMATION:
        vehicle = 'car1'
    if shape not in SHAPES:
        shape = 'spiral'
    gap0, lat0 = INITIAL_FORMATION[vehicle]
    gap1, lat1 = TARGET_FORMATION[vehicle]
    leader = SHAPES[shape]()
    count = len(leader)
    arc = [0.0]
    for i in range(1, count):
        arc.append(arc[-1] + math.dist(leader[i - 1], leader[i]))
    total = arc[-1]
    first = math.atan2(leader[1][1] - leader[0][1], leader[1][0] - leader[0][0])
    last = math.atan2(leader[-1][1] - leader[-2][1], leader[-1][0] - leader[-2][0])

    def sample(u):
        """Leader pose at arc length u; extrapolate straight before/after."""
        if u <= 0.0:
            x, y = leader[0]
            return x + u * math.cos(first), y + u * math.sin(first), first
        if u >= total:
            x, y = leader[-1]
            return x + (u - total) * math.cos(last), y + (u - total) * math.sin(last), last
        lo, hi = 0, count - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if arc[mid] <= u:
                lo = mid
            else:
                hi = mid
        t = (u - arc[lo]) / max(arc[hi] - arc[lo], 1e-9)
        x = leader[lo][0] + t * (leader[hi][0] - leader[lo][0])
        y = leader[lo][1] + t * (leader[hi][1] - leader[lo][1])
        h = math.atan2(leader[hi][1] - leader[lo][1], leader[hi][0] - leader[lo][0])
        return x, y, h

    world = []
    for i in range(count):
        s = i / (count - 1)
        w = s * s * (3.0 - 2.0 * s)  # smoothstep: no slot jump, no path kink
        gap = gap0 + (gap1 - gap0) * w
        lat = lat0 + (lat1 - lat0) * w
        x, y, h = sample(arc[i] - gap)
        world.append((x - lat * math.sin(h), y + lat * math.cos(h)))

    ox, oy = world[0]
    points = [(x - ox, y - oy) for x, y in world]
    result = []
    for i, (x, y) in enumerate(points):
        a, b = points[max(i - 1, 0)], points[min(i + 1, count - 1)]
        result.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0])))
    return result, (ox, oy)


def main(args=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from k7_msgs.action import FollowPath

    rclpy.init(args=args)
    node = Node('ref_path')
    try:
        shape = node.declare_parameter('shape', 'spiral').value
        if shape not in SHAPES:
            node.get_logger().error('unknown shape %s (choose %s)'
                                    % (shape, ', '.join(sorted(SHAPES))))
            return
        namespace = node.get_namespace().strip('/')
        client = ActionClient(node, FollowPath, 'task/follow_path')
        path = Path()
        path.header.frame_id = namespace + '/odom' if namespace else 'odom'
        points, _ = formation_path(namespace or 'car1', shape)
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)

        goal = FollowPath.Goal(path=path, target_speed=0.10,
                               position_tolerance=0.10, yaw_tolerance=0.15)
        client.wait_for_server()
        accepted = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, accepted)
        handle = accepted.result()
        if not handle or not handle.accepted:
            node.get_logger().error('Reference path goal rejected')
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        node.get_logger().info(f'Reference path finished ({shape}): '
                               f'{result.result().result.reason}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
