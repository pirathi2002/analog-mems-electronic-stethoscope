// Shared application state between the DSP task, the serial loop and BLE.
//
// Settings flow:  command (serial/BLE) -> app::requestSettings()
//                 DSP task             -> app::takePendingSettings() at the next
//                                         block boundary, re-designs its filters.
// Telemetry flow: DSP task publishes once per second -> status JSON / log line.
#pragma once

#include <stdint.h>

#include <atomic>

#include "../dsp/pcg_pipeline.h"

namespace app {

enum class StreamMode : uint8_t {
  kOff = 0,     // quiet (command replies only)
  kLog = 1,     // one human-readable status line per second (Serial Monitor)
  kPlot = 2,    // ASCII "raw:..,filtered:.." at 500 Hz (Arduino Serial Plotter)
  kBinary = 3,  // framed binary audio + JSON status (PC viewer / recorder)
};

const char* streamModeName(StreamMode m);

struct Telemetry {
  float fs_measured = 0;     // actual ADC sample rate (Hz)
  float rms_raw_dbfs = -120; // last 1 s, dB re ADC full scale
  float rms_filt_dbfs = -120;
  float peak_raw = 0;        // last 1 s, 0..1 of full scale
  int dc_mv = 0;             // mean AFE output voltage (should be ~1650 mV)
  uint32_t clip_total = 0;   // ADC samples at the rails since boot
  uint32_t clip_last = 0;    // ... in the last second
  uint32_t adc_overflows = 0;
  uint32_t sample_index = 0; // 4 kHz output sample counter
  dsp::HrResult hr;
};

struct Counters {
  std::atomic<uint32_t> serial_drops{0};  // audio blocks not sent on serial (host too slow)
  std::atomic<uint32_t> ble_drops{0};     // audio blocks not sent over BLE
};

void init(const dsp::PipelineSettings& s, bool ble_enabled);

void requestSettings(const dsp::PipelineSettings& s);
bool takePendingSettings(dsp::PipelineSettings* out);
dsp::PipelineSettings settings();  // latest requested settings

void publishTelemetry(const Telemetry& t);
Telemetry telemetry();

StreamMode streamMode();
void setStreamMode(StreamMode m);

bool bleEnabled();          // value stored in flash (applied at boot)
void setBleEnabled(bool on);
bool bleConnected();
void setBleConnected(bool on);

Counters& counters();

}  // namespace app
