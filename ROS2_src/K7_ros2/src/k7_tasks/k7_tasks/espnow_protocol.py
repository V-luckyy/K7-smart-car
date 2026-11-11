"""96-byte ESP-NOW/USB wire format. No ROS or serial dependency."""
import binascii
from dataclasses import dataclass
import math
import struct

SIZE = 96
VERSION = 1
FLEET = 1
STATUS = 2
_HEADER = struct.Struct('<2sBBHH')
_FLEET = struct.Struct('<2sBBHHQIiI16sdddffBB16sH')
_STATUS = struct.Struct('<2sBBHHI6sBBIIIII54sH')
assert _FLEET.size == _STATUS.size == SIZE


@dataclass(frozen=True)
class FleetPacket:
    sender_id: int
    boot_id: int
    sequence: int
    stamp_sec: int
    stamp_nsec: int
    session_id: bytes
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float
    state: int
    frame_id: str

    @property
    def stamp_ns(self):
        return self.stamp_sec * 1_000_000_000 + self.stamp_nsec


@dataclass(frozen=True)
class StatusPacket:
    sender_id: int
    uptime_ms: int
    mac: bytes
    role: int
    channel: int
    usb_rx: int
    radio_tx_done: int
    radio_tx_error: int
    radio_rx: int
    rejected: int


def crc16(data):
    return binascii.crc_hqx(data, 0xffff)


def _validate_fleet(packet):
    frame = packet.frame_id.encode('ascii')
    if (not 1 <= packet.sender_id <= 65535 or not 0 < packet.boot_id < 2**64 or
            not 0 <= packet.sequence < 2**32 or
            not 0 <= packet.stamp_sec < 2**31 or not 0 <= packet.stamp_nsec < 1_000_000_000 or
            len(packet.session_id) != 16 or not any(packet.session_id) or
            not 0 <= packet.state <= 3 or not 1 <= len(frame) <= 16 or b'\0' in frame or
            not all(math.isfinite(value) for value in (packet.x, packet.y, packet.yaw,
                                                       packet.linear_speed, packet.angular_speed))):
        raise ValueError('invalid FleetState wire values')
    return frame


def encode_fleet(packet):
    frame = _validate_fleet(packet)
    data = _FLEET.pack(b'K7', VERSION, FLEET, packet.sender_id, SIZE,
        packet.boot_id, packet.sequence, packet.stamp_sec, packet.stamp_nsec,
        packet.session_id, packet.x, packet.y, packet.yaw, packet.linear_speed,
        packet.angular_speed, packet.state, len(frame), frame.ljust(16, b'\0'), 0)
    return data[:-2] + struct.pack('<H', crc16(data[:-2]))


def decode(data):
    if len(data) != SIZE:
        raise ValueError('wrong frame size')
    magic, version, kind, sender, size = _HEADER.unpack_from(data)
    if (magic != b'K7' or version != VERSION or size != SIZE or sender == 0 or
            crc16(data[:-2]) != struct.unpack_from('<H', data, SIZE-2)[0]):
        raise ValueError('invalid header or CRC')
    if kind == FLEET:
        values = _FLEET.unpack(data)
        frame_length = values[16]
        if not 1 <= frame_length <= 16:
            raise ValueError('invalid frame_id length')
        packet = FleetPacket(sender, *values[5:16], values[17][:frame_length].decode('ascii'))
        _validate_fleet(packet)
        return packet
    if kind == STATUS:
        values = _STATUS.unpack(data)
        packet = StatusPacket(sender, *values[5:14])
        if packet.role not in (1, 2) or not 1 <= packet.channel <= 11:
            raise ValueError('invalid board role or channel')
        return packet
    raise ValueError('unsupported frame kind')


class StreamParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(b'K7')
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'K' else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < SIZE:
                break
            try:
                packet = decode(bytes(self.buffer[:SIZE]))
            except (ValueError, UnicodeError):
                self.bad_frames += 1
                del self.buffer[0]
                continue
            packets.append(packet)
            del self.buffer[:SIZE]
        return packets


class SourceOrder:
    """Source time and per-process sequence jointly reject old/duplicate state."""
    def __init__(self):
        self.boot_id = None
        self.sequence = 0
        self.stamp_ns = -1
        self.missing = 0

    def accept(self, packet):
        if packet.stamp_ns <= self.stamp_ns:
            return False
        if packet.boot_id == self.boot_id:
            delta = (packet.sequence - self.sequence) & 0xffffffff
            if not 0 < delta < 0x80000000:
                return False
            self.missing += delta - 1
        self.boot_id = packet.boot_id
        self.sequence = packet.sequence
        self.stamp_ns = packet.stamp_ns
        return True
