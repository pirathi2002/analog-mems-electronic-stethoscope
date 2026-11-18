#include "protocol.h"

#include <string.h>

namespace proto {

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static inline void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

static inline void put32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

size_t encodeFrame(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out, size_t out_cap) {
  if (len > 0xFFFF || len + kFrameOverhead > out_cap) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = type;
  put16(&out[3], static_cast<uint16_t>(len));
  if (len) memcpy(&out[5], payload, len);
  put16(&out[5 + len], crc16(&out[2], 3 + len));
  return len + kFrameOverhead;
}

size_t encodeAudioFrame(uint32_t first_index, const int16_t* raw, const int16_t* filt, uint16_t n,
                        uint8_t mode, uint8_t* out, size_t out_cap) {
  const size_t len = 8 + 4u * n;
  if (len + kFrameOverhead > out_cap) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = kFrameAudio;
  put16(&out[3], static_cast<uint16_t>(len));
  uint8_t* p = &out[5];
  put32(p, first_index);
  put16(p + 4, n);
  p[6] = 0x03;  // raw + filtered
  p[7] = mode;
  p += 8;
  for (uint16_t i = 0; i < n; ++i, p += 2) put16(p, static_cast<uint16_t>(raw[i]));
  for (uint16_t i = 0; i < n; ++i, p += 2) put16(p, static_cast<uint16_t>(filt[i]));
  put16(&out[5 + len], crc16(&out[2], 3 + len));
  return len + kFrameOverhead;
}

}  // namespace proto
