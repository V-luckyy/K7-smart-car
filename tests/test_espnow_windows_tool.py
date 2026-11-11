"""Windows test logic, plus end-to-end serial exercise on Linux PTYs."""
import binascii
from dataclasses import replace
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'ESP32_src/windows_test'
sys.path.insert(0, str(TOOL))
import windows_test as tool


def packet(sequence=0):
    return tool.FleetPacket(1, 99, sequence, 1700000000, 0, bytes(range(16)),
                            1.25, -0.25, 0.125, 0.05, -0.025, 0, 'fleet')


class VerificationTests(unittest.TestCase):
    def ready(self):
        result = tool.Verification()
        for i in range(15):
            p = packet(i)
            result.remember(p, tool.encode_fleet(p), i * 0.1)
            result.receive(p, i * 0.1 + 0.01, 'stream' if i < 10 else 'recovery', 0)
        self.assertTrue(result.result()['passed'])
        return result

    def test_expected_float32_rounding_is_accepted(self):
        from k7_tasks.espnow_protocol import decode
        result = tool.Verification()
        p = packet()
        data = tool.encode_fleet(p)
        result.remember(p, data, 1.0)
        result.receive(decode(data), 1.02, 'stream', 0)
        self.assertEqual(result.counts['received'], 1)
        self.assertAlmostEqual(result.latencies_ms[0], 20)

    def test_mismatch_unexpected_duplicate_and_pause_replay_fail(self):
        for name in ('mismatched', 'unexpected', 'duplicate', 'cached_during_pause'):
            result = self.ready()
            p = packet(15)
            if name == 'mismatched':
                result.remember(p, tool.encode_fleet(p), 1.5)
                result.receive(replace(p, x=7), 1.51, 'recovery', 0)
            elif name == 'unexpected':
                result.receive(p, 1.51, 'recovery', 0)
            elif name == 'duplicate':
                result.receive(packet(14), 1.51, 'recovery', 0)
            else:
                result.receive(packet(14), 1.51, 'pause', 1.5)
            self.assertGreater(result.counts[name], 0, name)
            self.assertFalse(result.result()['passed'], name)

    def test_sparse_or_stalled_delivery_fails(self):
        result = self.ready()
        for i in range(15, 30):
            p = packet(i)
            result.remember(p, tool.encode_fleet(p), i * 0.1)
        self.assertFalse(result.result()['passed'])
        result = self.ready()
        p = packet(15)
        result.remember(p, tool.encode_fleet(p), 2.5)
        result.receive(p, 2.6, 'recovery', 0)
        self.assertFalse(result.result()['passed'])

    def test_shell_help_needs_no_serial_hardware(self):
        result = subprocess.run(['bash', str(TOOL / 'run.sh'), '--help'],
                                capture_output=True, text=True, check=True)
        self.assertIn('--ports', result.stdout)
        self.assertIn('--duration', result.stdout)

    def test_generated_bundle_uses_current_protocol(self):
        import zipfile
        subprocess.run([sys.executable, str(TOOL / 'build_bundle.py')], check=True, capture_output=True)
        with zipfile.ZipFile(TOOL / 'k7-espnow-windows-test.zip') as archive:
            self.assertEqual(archive.read('k7-espnow-windows-test/espnow_protocol.py'),
                (ROOT / 'ROS2_src/K7_ros2/src/k7_tasks/k7_tasks/espnow_protocol.py').read_bytes())


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux PTY transport test')
class SerialRoundTripTests(unittest.TestCase):
    def test_discovery_send_pause_recovery_and_report_with_real_serial(self):
        import pty
        import serial
        pairs = [pty.openpty(), pty.openpty()]
        for master, slave in pairs:
            self.addCleanup(os.close, master)
            self.addCleanup(os.close, slave)
            os.set_blocking(master, False)
        stopped = threading.Event()
        errors = []

        def status(vehicle_id, role):
            payload = struct.pack('<2sBBHHI6sBBIIIII54s', b'K7', 1, 2, vehicle_id,
                96, 1234, bytes([0x24, 0x6f, 0x28, 0, 0, vehicle_id]), role, 6,
                0, 0, 0, 0, 0, bytes(54))
            return payload + struct.pack('<H', binascii.crc_hqx(payload, 0xffff))

        def virtual_firmware():
            parser = tool.StreamParser()
            next_status = 0
            try:
                while not stopped.is_set():
                    if time.monotonic() >= next_status:
                        for pair, frame in zip(pairs, (status(1, 1), status(2, 2))):
                            os.write(pair[0], frame)
                        next_status = time.monotonic() + 0.2
                    try:
                        data = os.read(pairs[0][0], 4096)
                    except BlockingIOError:
                        data = b''
                    for p in parser.feed(data):
                        if isinstance(p, tool.FleetPacket):
                            frame = tool.encode_fleet(p)
                            os.write(pairs[1][0], frame[:31])
                            os.write(pairs[1][0], frame[31:])
                    stopped.wait(0.005)
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=virtual_firmware)
        # Open first so PTY slave starts in raw serial mode, then publish status.
        boards = [tool.Board(serial, os.ttyname(slave)) for _, slave in pairs]
        for b in boards:
            b.close()
        thread.start()
        self.addCleanup(lambda: (stopped.set(), thread.join(timeout=2)))
        leader, follower = tool.discover(serial, [os.ttyname(s) for _, s in pairs], 2)
        self.addCleanup(leader.close)
        self.addCleanup(follower.close)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            self.assertTrue(tool.test_pair(leader, follower, SimpleNamespace(duration=2), output))
            report = json.loads((output / 'result.json').read_text())
            self.assertTrue(report['passed'])
            self.assertEqual(report['received_by_phase']['pause'], 0)
            self.assertEqual(report['counts']['sent'], report['counts']['received'])
            self.assertTrue((output / 'frames.jsonl').stat().st_size > 0)
        self.assertEqual(errors, [])


if __name__ == '__main__':
    unittest.main()
