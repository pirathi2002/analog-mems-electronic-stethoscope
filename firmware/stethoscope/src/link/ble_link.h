// Bluetooth Low Energy link (wireless operation on battery).
//
// GATT service BLE_SERVICE_UUID (see config.h):
//   AUDIO_RF (notify): uint32 first_sample_index + int16 raw[n] + int16 filtered[n] @ 4 kHz,
//                      n = (packet_len - 4) / 4. A 64-sample block is sent as 2 x 32 samples
//                      when MTU >= 135 (smaller pieces otherwise). 128 kbit/s.
//   AUDIO    (notify): uint32 first_sample_index + int16 filtered samples @ 4 kHz (64 kbit/s).
//                      Legacy filtered-only stream, sent only while AUDIO_RF is not subscribed.
//   STATUS  (notify, read) : UTF-8. JSON status once per second ("{...}") or a
//                      plain-text reply to a command.
//   CONTROL (write)  : text commands, same as on the serial port ("mode bell").
//
// PC client: firmware/tools/stetho/transport.py (BleTransport, uses bleak).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ble_link {

bool begin();

// Non-blocking; called from the DSP task. Drops the block if the queue is full.
void pushAudio(uint32_t first_index, const int16_t* raw, const int16_t* filt, uint16_t n);

// Called once per second from the main loop.
void publishStatus(const char* text, size_t len);

}  // namespace ble_link
