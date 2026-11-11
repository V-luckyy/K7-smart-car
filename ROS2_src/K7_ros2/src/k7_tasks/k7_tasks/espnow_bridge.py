"""Leader FleetState -> USB; follower USB -> local FleetState topic."""
import secrets
import time

import rclpy
from rclpy.node import Node
import serial
from diagnostic_msgs.msg import DiagnosticArray
from k7_msgs.msg import FleetState
from k7_control.ros_utils import diagnostic, parameters
from .espnow_protocol import FleetPacket, SourceOrder, StatusPacket, StreamParser, encode_fleet


class EspnowBridge(Node):
    def __init__(self):
        super().__init__('espnow_bridge')
        self.p = parameters(self, dict(role='leader', vehicle_id=1, leader_id=1,
            device='/dev/k7_espnow', baud=115200, channel=6, fleet_frame='fleet', max_age=0.35))
        p = self.p
        if (p['role'] not in ('leader', 'follower') or
                not 1 <= p['vehicle_id'] <= 65535 or not 1 <= p['leader_id'] <= 65535 or
                (p['role'] == 'leader') != (p['vehicle_id'] == p['leader_id']) or
                not 1 <= p['channel'] <= 11 or not 0 < p['max_age'] <= 0.5 or
                not 1 <= len(p['fleet_frame'].encode('ascii')) <= 16):
            raise ValueError('invalid ESP-NOW role, IDs, channel, max_age or fleet_frame')
        self.port = None
        self.parser = StreamParser()
        self.order = SourceOrder()
        self.boot_id = secrets.randbits(64) or 1
        self.sequence = 0
        self.pending = None
        self.last_connect = self.last_tx = self.last_rx = self.status_time = -float('inf')
        self.board = None
        self.usb_tx = self.usb_rx = self.dropped = 0
        self.status_pub = self.create_publisher(DiagnosticArray, 'fleet/link', 1)
        self.fleet_pub = None
        if p['role'] == 'leader':
            self.create_subscription(FleetState, 'fleet/state', self.source_cb, 1)
        else:
            self.fleet_pub = self.create_publisher(FleetState, 'fleet/leader_state', 1)
        self.create_timer(0.01, self.pump)
        self.create_timer(1.0, self.publish_status)

    def ready(self, now):
        return self.port is not None and now - self.status_time <= 2.5

    def fresh(self, packet):
        age = (self.get_clock().now().nanoseconds - packet.stamp_ns) / 1e9
        return -0.05 <= age <= self.p['max_age']

    def source_cb(self, msg):
        # Latest-only handoff; pump enforces the radio's 10 Hz budget.
        self.pending = msg

    def disconnect(self):
        if self.port is not None:
            self.port.close()
        self.port = None
        self.pending = None
        self.status_time = -float('inf')
        self.parser = StreamParser()

    def connect(self, now):
        if now - self.last_connect < 1.0:
            return
        self.last_connect = now
        port = serial.Serial(port=None, baudrate=self.p['baud'], timeout=0,
                             write_timeout=0.02, exclusive=True)
        # Set levels before opening to reduce auto-reset on development boards.
        port.dtr = False
        port.rts = False
        port.port = self.p['device']
        try:
            port.open()
            port.reset_input_buffer()
        except (OSError, serial.SerialException):
            port.close()
            raise
        self.port = port
        self.get_logger().info('USB connected: ' + self.p['device'])

    def receive(self, packet, now):
        if isinstance(packet, StatusPacket):
            expected_role = 1 if self.p['role'] == 'leader' else 2
            if (packet.sender_id != self.p['vehicle_id'] or packet.role != expected_role or
                    packet.channel != self.p['channel']):
                self.status_time = -float('inf')
                self.get_logger().error('ESP32 role/vehicle_id/channel differs from K7 configuration',
                                        throttle_duration_sec=5.0)
                return
            self.board, self.status_time = packet, now
            return
        if (self.fleet_pub is None or not self.ready(now) or
                packet.sender_id != self.p['leader_id'] or
                packet.frame_id != self.p['fleet_frame'] or not self.fresh(packet) or
                not self.order.accept(packet)):
            self.dropped += 1
            return
        msg = FleetState()
        msg.header.stamp.sec = packet.stamp_sec
        msg.header.stamp.nanosec = packet.stamp_nsec
        msg.header.frame_id = packet.frame_id
        msg.session_id.uuid = list(packet.session_id)
        msg.x, msg.y, msg.yaw = packet.x, packet.y, packet.yaw
        msg.linear_speed, msg.angular_speed = packet.linear_speed, packet.angular_speed
        msg.state = packet.state
        self.fleet_pub.publish(msg)
        self.last_rx = now
        self.usb_rx += 1

    def transmit(self, now):
        if self.pending is None or not self.ready(now) or now - self.last_tx < 0.1:
            return
        msg, self.pending = self.pending, None
        packet = FleetPacket(self.p['vehicle_id'], self.boot_id, self.sequence,
            msg.header.stamp.sec, msg.header.stamp.nanosec, bytes(msg.session_id.uuid),
            msg.x, msg.y, msg.yaw, msg.linear_speed, msg.angular_speed,
            msg.state, msg.header.frame_id)
        try:
            data = encode_fleet(packet)
        except (ValueError, UnicodeError, OverflowError) as exc:
            self.dropped += 1
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return
        if packet.frame_id != self.p['fleet_frame'] or not self.fresh(packet):
            self.dropped += 1
            return
        written = self.port.write(data)
        if written != len(data):
            raise serial.SerialTimeoutException('partial USB frame write')
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.usb_tx += 1
        self.last_tx = now

    def pump(self):
        now = time.monotonic()
        try:
            if self.port is None:
                self.connect(now)
            if self.port is None:
                return
            chunk = self.port.read(min(self.port.in_waiting, 1536))
            for packet in self.parser.feed(chunk):
                self.receive(packet, now)
            if self.p['role'] == 'leader':
                self.transmit(now)
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warning('USB link: ' + str(exc), throttle_duration_sec=5.0)
            self.disconnect()

    def publish_status(self):
        now = time.monotonic()
        ok = self.ready(now)
        fresh_rx = now - self.last_rx <= self.p['max_age']
        level = 2 if not ok else 1 if self.fleet_pub and not fresh_rx else 0
        values = dict(role=self.p['role'], device=self.p['device'], usb_ready=ok,
            usb_tx=self.usb_tx, fleet_rx=self.usb_rx, dropped=self.dropped,
            sequence_gaps=self.order.missing, framing_errors=self.parser.bad_frames,
            receive_age_s=round(now-self.last_rx, 3) if self.last_rx > 0 else -1)
        if self.board is not None:
            board = self.board
            values.update(sta_mac=':'.join(f'{b:02X}' for b in board.mac),
                firmware_vehicle_id=board.sender_id, channel=board.channel,
                uptime_ms=board.uptime_ms, firmware_usb_rx=board.usb_rx,
                radio_tx_done=board.radio_tx_done, radio_tx_error=board.radio_tx_error,
                radio_rx=board.radio_rx, firmware_rejected=board.rejected)
        message = ('USB/firmware status timeout' if not ok else
                   'leader state timeout' if self.fleet_pub and not fresh_rx else 'ready')
        self.status_pub.publish(diagnostic(self, 'espnow_bridge', values, level, message))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EspnowBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()
