// Binary framing used on the serial link (device -> PC).
//
//   offset size  field
//   0      1     0xA5  sync
//   1      1     0x5A  sync
//   2      1     type  (FrameType)
//   3      2     len   payload length, little-endian
//   5      len   payload
//   5+len  2     CRC-16/CCITT-FALSE over bytes [2 .. 5+len), little-endian
//
// AUDIO payload (type 0x01):
//   uint32 first_sample_index   (4 kHz sample counter, detects lost blocks)
//   uint16 n                    samples per channel
//   uint8  channels             bit0 = raw, bit1 = filtered
//   uint8  mode                 dsp::FilterMode at the time of the block
//   int16  raw[n]               full scale +/-32767 = ADC full scale
//   int16  filtered[n]
//
// STATUS (0x02) and TEXT (0x03) payloads are UTF-8: STATUS is a JSON object,
// TEXT is a human-readable reply to a command.
//
// PC -> device: plain ASCII command lines terminated by '\n' (see commands.cpp).
// Decoder: firmware/tools/stetho/protocol.py
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace proto {

enum FrameType : uint8_t {
  kFrameAudio = 0x01,
  kFrameStatus = 0x02,
  kFrameText = 0x03,
};

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr size_t kFrameOverhead = 7;

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// Writes a complete frame into out (capacity out_cap). Returns the frame
// length, or 0 if it does not fit.
size_t encodeFrame(uint8_t type, const uint8_t* payload, size_t len, uint8_t* out, size_t out_cap);

// Builds an AUDIO frame directly from two int16 channels.
size_t encodeAudioFrame(uint32_t first_index, const int16_t* raw, const int16_t* filt, uint16_t n,
                        uint8_t mode, uint8_t* out, size_t out_cap);

}  // namespace proto
