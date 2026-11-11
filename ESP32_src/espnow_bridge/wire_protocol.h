#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace k7 {
constexpr size_t FRAME_SIZE = 96;
constexpr uint8_t VERSION = 1;
constexpr uint8_t FLEET = 1;
constexpr uint8_t STATUS = 2;

// Wire integers are little-endian; floats use IEEE754. ESP32 and K7 ARM64
// match this representation. Python uses explicit '<' packing, never native layout.
#pragma pack(push, 1)
struct Header {
  uint8_t magic[2];
  uint8_t version;
  uint8_t kind;
  uint16_t sender_id;
  uint16_t size;
};

struct FleetFrame {
  Header header;
  uint64_t boot_id;       // K7 bridge process identity, distinct from task session.
  uint32_t sequence;
  int32_t stamp_sec;      // Original ROS sample timestamp; never restamped by ESP32.
  uint32_t stamp_nsec;
  uint8_t session_id[16];
  double x, y, yaw;       // Common fleet frame: metres, metres, radians.
  float linear_speed;    // Body forward speed, m/s.
  float angular_speed;   // Counterclockwise yaw rate, rad/s.
  uint8_t state;
  uint8_t frame_length;
  char frame_id[16];
  uint16_t crc;
};

struct StatusFrame {
  Header header;
  uint32_t uptime_ms;
  uint8_t mac[6];
  uint8_t role;           // 1 leader, 2 follower.
  uint8_t channel;
  uint32_t usb_rx;
  uint32_t radio_tx_done; // Broadcast completion, not per-follower delivery.
  uint32_t radio_tx_error;
  uint32_t radio_rx;
  uint32_t rejected;
  uint8_t reserved[54];
  uint16_t crc;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 8, "header layout changed");
static_assert(sizeof(FleetFrame) == FRAME_SIZE, "fleet layout changed");
static_assert(sizeof(StatusFrame) == FRAME_SIZE, "status layout changed");
static_assert(sizeof(double) == 8 && sizeof(float) == 4, "IEEE754 sizes required");

inline Header header(uint8_t kind, uint16_t id) {
  return Header{{'K', '7'}, VERSION, kind, id, FRAME_SIZE};
}

// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xffff, xorout 0.
inline uint16_t crc16(const uint8_t* data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = crc & 0x8000 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

inline void seal(void* frame) {
  auto* data = static_cast<uint8_t*>(frame);
  const uint16_t crc = crc16(data, FRAME_SIZE - 2);
  data[94] = static_cast<uint8_t>(crc);
  data[95] = static_cast<uint8_t>(crc >> 8);
}

inline bool valid(const uint8_t* data, size_t size) {
  return size == FRAME_SIZE && data[0] == 'K' && data[1] == '7' &&
      data[2] == VERSION && (data[3] == FLEET || data[3] == STATUS) &&
      data[6] == FRAME_SIZE && data[7] == 0 &&
      crc16(data, FRAME_SIZE - 2) == (data[94] | (uint16_t(data[95]) << 8));
}

class StreamParser {
 public:
  // A failed candidate slides one byte, recovering from boot text, corruption,
  // truncation and concatenated USB reads without assuming read boundaries.
  bool push(uint8_t value, FleetFrame& output) {
    buffer_[used_++] = value;
    if (used_ < FRAME_SIZE) return false;
    if (valid(buffer_, FRAME_SIZE)) {
      memcpy(&output, buffer_, FRAME_SIZE);
      used_ = 0;
      return true;
    }
    memmove(buffer_, buffer_ + 1, FRAME_SIZE - 1);
    used_ = FRAME_SIZE - 1;
    return false;
  }
 private:
  uint8_t buffer_[FRAME_SIZE]{};
  size_t used_ = 0;
};
}  // namespace k7
