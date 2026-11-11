"""Real Python/C++ wire interoperability and serial framing tests."""
from dataclasses import replace
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ROS2_src/K7_ros2/src/k7_tasks'))
from k7_tasks.espnow_protocol import (FleetPacket, SIZE, SourceOrder, StatusPacket,
                                     StreamParser, crc16, decode, encode_fleet)


def sample(**changes):
    packet = FleetPacket(1, 0x0807060504030201, 42, 1700000000, 123456789,
                         bytes(range(16)), 1.25, -2.5, 0.5, 0.25, -0.125, 1, 'fleet')
    return replace(packet, **changes)


CPP = r'''
#include <iostream>
#include <iterator>
#include <vector>
#include "wire_protocol.h"
int main(int argc, char**) {
  if (argc > 1) {
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(std::cin)), {});
    k7::StreamParser parser;
    k7::FleetFrame output;
    for (uint8_t value : bytes)
      if (parser.push(value, output))
        std::cout.write(reinterpret_cast<const char*>(&output), sizeof(output));
    return 0;
  }
  k7::FleetFrame frame{};
  frame.header = k7::header(k7::FLEET, 1);
  frame.boot_id = 0x0807060504030201ULL;
  frame.sequence = 42;
  frame.stamp_sec = 1700000000;
  frame.stamp_nsec = 123456789;
  for (int i = 0; i < 16; ++i) frame.session_id[i] = i;
  frame.x = 1.25; frame.y = -2.5; frame.yaw = 0.5;
  frame.linear_speed = 0.25f; frame.angular_speed = -0.125f;
  frame.state = 1; frame.frame_length = 5;
  memcpy(frame.frame_id, "fleet", 5);
  k7::seal(&frame);
  std::cout.write(reinterpret_cast<const char*>(&frame), sizeof(frame));
  k7::StatusFrame status{};
  status.header = k7::header(k7::STATUS, 2);
  status.uptime_ms = 1234;
  const uint8_t mac[] = {0x24, 0x6f, 0x28, 0x01, 0x02, 0x03};
  memcpy(status.mac, mac, 6);
  status.role = 2; status.channel = 6;
  status.usb_rx = 1; status.radio_tx_done = 2; status.radio_tx_error = 3;
  status.radio_rx = 4; status.rejected = 5;
  k7::seal(&status);
  std::cout.write(reinterpret_cast<const char*>(&status), sizeof(status));
}
'''


class ProtocolTests(unittest.TestCase):
    def test_crc_standard_check_value(self):
        # Published CRC-16/CCITT-FALSE check vector, independent of our encoder.
        self.assertEqual(crc16(b'123456789'), 0x29b1)

    def test_fixed_offsets_and_roundtrip(self):
        data = encode_fleet(sample())
        self.assertEqual(len(data), 96)
        self.assertEqual(data[:20], bytes.fromhex(
            '4b3701010100600001020304050607082a000000'))
        self.assertEqual(data[44:52], bytes.fromhex('000000000000f43f'))
        self.assertEqual(data[52:60], bytes.fromhex('00000000000004c0'))
        self.assertEqual(data[76:83], b'\x01\x05fleet')
        self.assertEqual(decode(data), sample())

    def test_entry_validation(self):
        for changes in (dict(x=float('nan')), dict(angular_speed=float('inf')),
                        dict(state=4), dict(stamp_nsec=1_000_000_000),
                        dict(frame_id='x'*17), dict(session_id=bytes(16)), dict(sender_id=0)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                encode_fleet(sample(**changes))

    def test_corruption_truncation_and_bad_version(self):
        data = encode_fleet(sample())
        with self.assertRaises(ValueError):
            decode(data[:-1])
        for offset in range(SIZE):
            broken = bytearray(data)
            broken[offset] ^= 1
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                decode(broken)
        broken = bytearray(data)
        broken[2] = 2
        broken[-2:] = struct.pack('<H', crc16(broken[:-2]))
        with self.assertRaises(ValueError):
            decode(broken)

    def test_serial_noise_fragmentation_and_recovery(self):
        first = encode_fleet(sample())
        second = encode_fleet(sample(sequence=43, stamp_nsec=223456789))
        broken = bytearray(first)
        broken[50] ^= 0x80
        stream = b'ESP-ROM boot\r\nK' + first[:39] + bytes(broken) + first + second
        parser = StreamParser()
        result = []
        rng = random.Random(7)
        while stream:
            width = rng.randint(1, 71)
            result.extend(parser.feed(stream[:width]))
            stream = stream[width:]
        self.assertEqual(result, [sample(), sample(sequence=43, stamp_nsec=223456789)])
        self.assertGreater(parser.bad_frames, 0)
        parser.feed(b'noise' * 10000)
        self.assertLess(len(parser.buffer), SIZE)

    def test_order_duplicates_gaps_wrap_and_bridge_restart(self):
        order = SourceOrder()
        self.assertTrue(order.accept(sample(sequence=0xfffffffe)))
        self.assertFalse(order.accept(sample(sequence=0xfffffffe, stamp_nsec=223456789)))
        self.assertTrue(order.accept(sample(sequence=0, stamp_nsec=223456789)))
        self.assertEqual(order.missing, 1)
        self.assertFalse(order.accept(sample(sequence=0xffffffff, stamp_nsec=323456789)))
        self.assertTrue(order.accept(sample(boot_id=99, sequence=0, stamp_nsec=423456789)))
        self.assertFalse(order.accept(sample(sequence=1, stamp_nsec=323456789)))
        self.assertTrue(order.accept(sample(boot_id=99, sequence=1, stamp_nsec=523456789)))


@unittest.skipUnless(shutil.which('g++'), 'host C++ compiler unavailable')
class FirmwareWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        source = Path(cls.temp.name) / 'wire.cpp'
        source.write_text(CPP)
        cls.executable = str(Path(cls.temp.name) / 'wire')
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
            '-I', str(ROOT / 'ESP32_src/espnow_bridge'), str(source), '-o', cls.executable], check=True)

    def test_firmware_fleet_bytes_match_python(self):
        data = subprocess.check_output([self.executable])
        self.assertEqual(data[:SIZE], encode_fleet(sample()))
        self.assertEqual(decode(data[SIZE:]), StatusPacket(
            2, 1234, bytes.fromhex('246f28010203'), 2, 6, 1, 2, 3, 4, 5))

    def test_firmware_parser_recovers_python_usb_stream(self):
        data = encode_fleet(sample())
        broken = bytearray(data)
        broken[35] ^= 2
        stream = b'boot\r\n' + data[:37] + bytes(broken) + data + data
        output = subprocess.check_output([self.executable, 'parse'], input=stream)
        self.assertEqual(output, data + data)


if __name__ == '__main__':
    unittest.main()
