#include "app_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace app {

namespace {
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
dsp::PipelineSettings g_settings;
dsp::PipelineSettings g_pending;
bool g_has_pending = false;
Telemetry g_telemetry;
volatile StreamMode g_stream = StreamMode::kLog;
volatile bool g_ble_enabled = true;
volatile bool g_ble_connected = false;
Counters g_counters;
}  // namespace

const char* streamModeName(StreamMode m) {
  switch (m) {
    case StreamMode::kOff: return "off";
    case StreamMode::kLog: return "log";
    case StreamMode::kPlot: return "plot";
    case StreamMode::kBinary: return "bin";
  }
  return "?";
}

void init(const dsp::PipelineSettings& s, bool ble_enabled) {
  portENTER_CRITICAL(&g_mux);
  g_settings = s;
  g_pending = s;
  g_has_pending = true;
  portEXIT_CRITICAL(&g_mux);
  g_ble_enabled = ble_enabled;
}

void requestSettings(const dsp::PipelineSettings& s) {
  portENTER_CRITICAL(&g_mux);
  g_settings = s;
  g_pending = s;
  g_has_pending = true;
  portEXIT_CRITICAL(&g_mux);
}

bool takePendingSettings(dsp::PipelineSettings* out) {
  bool has = false;
  portENTER_CRITICAL(&g_mux);
  if (g_has_pending) {
    *out = g_pending;
    g_has_pending = false;
    has = true;
  }
  portEXIT_CRITICAL(&g_mux);
  return has;
}

dsp::PipelineSettings settings() {
  portENTER_CRITICAL(&g_mux);
  dsp::PipelineSettings s = g_settings;
  portEXIT_CRITICAL(&g_mux);
  return s;
}

void publishTelemetry(const Telemetry& t) {
  portENTER_CRITICAL(&g_mux);
  g_telemetry = t;
  portEXIT_CRITICAL(&g_mux);
}

Telemetry telemetry() {
  portENTER_CRITICAL(&g_mux);
  Telemetry t = g_telemetry;
  portEXIT_CRITICAL(&g_mux);
  return t;
}

StreamMode streamMode() { return g_stream; }
void setStreamMode(StreamMode m) { g_stream = m; }
bool bleEnabled() { return g_ble_enabled; }
void setBleEnabled(bool on) { g_ble_enabled = on; }
bool bleConnected() { return g_ble_connected; }
void setBleConnected(bool on) { g_ble_connected = on; }
Counters& counters() { return g_counters; }

}  // namespace app
