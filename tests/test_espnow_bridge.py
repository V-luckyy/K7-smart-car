"""Bridge behavior with real wire packets and a real Linux pseudo-terminal.

Only ROS clock/message/publisher boundaries are replaced on hosts without ROS.
"""
import ast
from dataclasses import replace
from pathlib import Path
import os
import pty
import sys
import time
from types import SimpleNamespace
import unittest

import serial

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ROS2_src/K7_ros2/src/k7_tasks'))
from k7_tasks.espnow_protocol import (FleetPacket, SourceOrder, StatusPacket,
                                     StreamParser, decode)


def message():
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=''),
                           session_id=SimpleNamespace(uuid=[]))


tree = ast.parse((ROOT / 'ROS2_src/K7_ros2/src/k7_tasks/k7_tasks/espnow_bridge.py').read_text())
klass = next(item for item in tree.body if isinstance(item, ast.ClassDef))
klass.bases = []
scope = dict(time=time, serial=serial, FleetPacket=FleetPacket, StatusPacket=StatusPacket,
             FleetState=message, StreamParser=StreamParser)
from k7_tasks.espnow_protocol import encode_fleet
scope['encode_fleet'] = encode_fleet
exec(compile(ast.Module(body=[klass], type_ignores=[]), '<bridge methods>', 'exec'), scope)


class BridgeTests(unittest.TestCase):
    def node(self):
        node = scope['EspnowBridge'].__new__(scope['EspnowBridge'])
        node.p = dict(role='follower', leader_id=1, vehicle_id=2, fleet_frame='fleet',
                      max_age=0.35, channel=6)
        node.port = object()
        node.order = SourceOrder()
        node.parser = StreamParser()
        node.status_time = 10.0
        node.last_rx = -float('inf')
        node.usb_rx = node.dropped = 0
        node.board = None
        node.output = []
        node.fleet_pub = SimpleNamespace(publish=node.output.append)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_700_000_000_500_000_000))
        node.get_logger = lambda: SimpleNamespace(error=lambda *args, **kw: None,
                                                  warning=lambda *args, **kw: None)
        return node

    def packet(self):
        return FleetPacket(1, 123, 7, 1700000000, 400000000, bytes(range(16)),
                           1.0, -2.0, 0.5, 0.2, 0.1, 1, 'fleet')

    def test_forward_preserves_source_stamp_session_and_all_fields(self):
        node = self.node()
        packet = self.packet()
        node.receive(packet, 10.1)
        msg, = node.output
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec),
                         (packet.stamp_sec, packet.stamp_nsec))
        self.assertEqual(msg.header.frame_id, 'fleet')
        self.assertEqual(bytes(msg.session_id.uuid), packet.session_id)
        self.assertEqual((msg.x, msg.y, msg.yaw, msg.linear_speed, msg.angular_speed, msg.state),
                         (1.0, -2.0, 0.5, 0.2, 0.1, 1))

    def test_stale_future_wrong_sender_frame_and_offline_status_are_dropped(self):
        for changes in (dict(stamp_nsec=100000000), dict(stamp_nsec=600000000),
                        dict(sender_id=3), dict(frame_id='car1/odom')):
            node = self.node()
            node.receive(replace(self.packet(), **changes), 10.1)
            self.assertEqual(node.output, [])
            self.assertEqual(node.last_rx, -float('inf'))
        node = self.node()
        node.receive(self.packet(), 13.0)
        self.assertEqual(node.output, [])

    def test_duplicate_does_not_extend_freshness(self):
        node = self.node()
        node.receive(self.packet(), 10.1)
        node.receive(self.packet(), 10.2)
        self.assertEqual(len(node.output), 1)
        self.assertEqual(node.last_rx, 10.1)

    def test_status_checks_local_board_role_id_and_channel(self):
        packet = StatusPacket(2, 1000, bytes(6), 2, 6, 0, 0, 0, 0, 0)
        node = self.node()
        for changes in (dict(role=1), dict(sender_id=3), dict(channel=1)):
            node.receive(replace(packet, **changes), 10.1)
            self.assertFalse(node.ready(10.1))
        node.receive(packet, 10.2)
        self.assertTrue(node.ready(10.2))

    def test_leader_serial_output_uses_real_pty_and_rate_limit(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        port = serial.Serial(os.ttyname(slave), 115200, timeout=0, write_timeout=0.02)
        self.addCleanup(port.close)
        node = self.node()
        node.port = port
        node.p['vehicle_id'] = 1
        node.boot_id = 123
        node.sequence = 7
        node.last_tx = 0.0
        node.usb_tx = 0
        source = message()
        packet = self.packet()
        source.header.stamp.sec, source.header.stamp.nanosec = packet.stamp_sec, packet.stamp_nsec
        source.header.frame_id = packet.frame_id
        source.session_id.uuid = list(packet.session_id)
        for field in ('x', 'y', 'yaw', 'linear_speed', 'angular_speed', 'state'):
            setattr(source, field, getattr(packet, field))
        node.source_cb(source)
        node.transmit(10.1)
        decoded = decode(os.read(master, 96))
        self.assertEqual(decoded.sequence, 7)
        self.assertEqual(decoded.stamp_ns, packet.stamp_ns)
        self.assertEqual(decoded.session_id, packet.session_id)
        self.assertEqual(node.usb_tx, 1)
        node.source_cb(source)
        node.transmit(10.15)
        self.assertIs(node.pending, source)
        self.assertEqual(node.usb_tx, 1)


if __name__ == '__main__':
    unittest.main()
