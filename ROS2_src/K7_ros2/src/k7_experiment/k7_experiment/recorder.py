import csv
import hashlib
import json
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import SetBool


class Recorder(Node):
    def __init__(self):
        super().__init__('experiment_recorder')
        defaults = dict(enabled=False, output_dir='logs', config_file='', config_sha256='', profile='',
            algorithm='', test_mode='none', rate=5.0, max_seconds=1800.0, min_free_mb=512,
            variables=['x', 'y', 'v', 'w', 'v_cmd', 'w_cmd'],
            topics=['odom', 'cmd_vel', 'task/status', 'control/status'])
        descriptor = ParameterDescriptor(read_only=True)
        self.p = {k: self.declare_parameter(k, v, descriptor).value for k, v in defaults.items()}
        self.config_bytes = Path(self.p['config_file']).read_bytes() if self.p['config_file'] else None
        if self.p['config_sha256'] and (self.config_bytes is None or
                hashlib.sha256(self.config_bytes).hexdigest() != self.p['config_sha256']):
            raise ValueError('configuration changed since launch; stop and restart before recording')
        allowed = {'x', 'y', 'v', 'w', 'v_cmd', 'w_cmd', 'cte', 'heading_error',
                   'w_stanley', 'w_apf', 'solve_ms'}
        if (not set(self.p['variables']) <= allowed or not 0 < self.p['rate'] <= 20
                or self.p['max_seconds'] <= 0 or self.p['min_free_mb'] < 64):
            raise ValueError('invalid recording fields, rate, duration or disk reserve')
        if any(not t or t.startswith('-') or any(c.isspace() for c in t) for t in self.p['topics']):
            raise ValueError('record topics must be explicit ROS names')
        self.file = self.process = self.console = None
        self.subscriptions_active = []
        self.values = {}
        self.last_health = 0
        self.create_service(SetBool, 'experiment/record', self.toggle)
        self.create_timer(1/self.p['rate'], self.tick)
        if self.p['enabled']:
            self.start()

    def start(self):
        if self.file:
            return
        root = Path(self.p['output_dir']).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(root).free < self.p['min_free_mb']*1024**2:
            raise OSError('insufficient recording disk reserve')
        name = time.strftime('%Y%m%d-%H%M%S')+'-'+self.get_namespace().strip('/').replace('/', '_')+'-'+uuid.uuid4().hex[:8]
        self.directory = root/name
        self.directory.mkdir()
        self.started = time.monotonic()
        self.values.clear()
        self.manifest = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            namespace=self.get_namespace(), settings=self.p, status='recording')
        if self.config_bytes is not None:
            data = self.config_bytes
            (self.directory/'config.yaml').write_bytes(data)
            self.manifest['config_sha256'] = hashlib.sha256(data).hexdigest()
        (self.directory/'manifest.json').write_text(json.dumps(self.manifest, indent=2), encoding='utf-8')
        try:
            self.file = (self.directory/'metrics.csv').open('w', newline='', encoding='utf-8')
            self.csv = csv.writer(self.file)
            self.csv.writerow(['ros_time']+self.p['variables'])
            wanted = set(self.p['variables'])
            for topic, cls, callback, fields in [
                ('odom', Odometry, self.odom, {'x','y','v','w'}),
                ('cmd_vel', TwistStamped, self.command, {'v_cmd','w_cmd'}),
                ('control/debug', DiagnosticArray, self.debug, {'cte','heading_error','w_stanley','w_apf','solve_ms'})]:
                if wanted & fields:
                    self.subscriptions_active.append(self.create_subscription(cls, topic, callback, 1))
            if self.p['topics']:
                self.console = (self.directory/'rosbag.log').open('wb')
                topics = [self.resolve_topic_name(t) for t in self.p['topics']]
                self.process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(self.directory/'bag'),
                    '--max-cache-size', '16777216', *topics], stdout=self.console, stderr=subprocess.STDOUT)
        except Exception:
            self.stop('start failed')
            raise

    def stop(self, reason='requested'):
        if self.process:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
                    reason += '; rosbag forced exit, check bag integrity'
            self.process = None
        if self.console:
            self.console.close()
            self.console = None
        for sub in self.subscriptions_active:
            self.destroy_subscription(sub)
        self.subscriptions_active.clear()
        if self.file:
            self.file.close()
            self.file = None
            self.manifest.update(status='closed', reason=reason, seconds=time.monotonic()-self.started)
            (self.directory/'manifest.json').write_text(json.dumps(self.manifest, indent=2), encoding='utf-8')

    def toggle(self, request, response):
        try:
            self.start() if request.data else self.stop()
            response.success, response.message = True, 'recording' if self.file else 'disabled'
        except (OSError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def put(self, **values):
        now = time.monotonic()
        self.values.update({k: (v, now) for k, v in values.items()})

    def odom(self, msg):
        self.put(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                 v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z)

    def command(self, msg):
        self.put(v_cmd=msg.twist.linear.x, w_cmd=msg.twist.angular.z)

    def debug(self, msg):
        self.put(**{kv.key: kv.value for status in msg.status for kv in status.values})

    def tick(self):
        if not self.file:
            return
        now = time.monotonic()
        try:
            if now-self.last_health > 1:
                self.last_health = now
                if (now-self.started >= self.p['max_seconds'] or
                        shutil.disk_usage(self.directory).free < self.p['min_free_mb']*1024**2 or
                        (self.process and self.process.poll() is not None)):
                    self.stop('duration, disk reserve or rosbag exit')
                    return
                self.file.flush()
            self.csv.writerow([self.get_clock().now().nanoseconds/1e9] +
                [self.values[k][0] if k in self.values and now-self.values[k][1] <= 0.5 else ''
                 for k in self.p['variables']])
        except OSError as exc:
            self.get_logger().error(str(exc))
            self.stop('write failure')


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.stop('shutdown')
        node.destroy_node()
        rclpy.try_shutdown()
