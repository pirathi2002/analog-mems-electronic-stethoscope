#include "settings_store.h"

#include <Preferences.h>

#include "../config.h"

namespace settings_store {

namespace {
constexpr const char* kNamespace = "stetho";
constexpr uint8_t kVersion = 1;  // bump when the stored layout changes
}  // namespace

dsp::PipelineSettings defaults() {
  dsp::PipelineSettings s;
  s.mode = static_cast<dsp::FilterMode>(DEFAULT_MODE);
  s.notch_hz = DEFAULT_NOTCH_HZ;
  s.gain = DEFAULT_GAIN;
  s.order = DEFAULT_ORDER;
  return s;
}

dsp::PipelineSettings load(bool* ble_enabled) {
  dsp::PipelineSettings s = defaults();
  *ble_enabled = BLE_ENABLED_DEFAULT;
  Preferences p;
  if (!p.begin(kNamespace, true)) return s;
  if (p.getUChar("ver", 0) == kVersion) {
    const uint8_t m = p.getUChar("mode", DEFAULT_MODE);
    if (m < dsp::kNumFilterModes) s.mode = static_cast<dsp::FilterMode>(m);
    s.custom_hp_hz = p.getFloat("hp", s.custom_hp_hz);
    s.custom_lp_hz = p.getFloat("lp", s.custom_lp_hz);
    s.notch_hz = p.getFloat("notch", s.notch_hz);
    s.gain = p.getFloat("gain", s.gain);
    s.order = p.getUChar("order", s.order);
    *ble_enabled = p.getBool("ble", *ble_enabled);
  }
  p.end();
  return s;
}

bool save(const dsp::PipelineSettings& s, bool ble_enabled) {
  Preferences p;
  if (!p.begin(kNamespace, false)) return false;
  p.putUChar("ver", kVersion);
  p.putUChar("mode", static_cast<uint8_t>(s.mode));
  p.putFloat("hp", s.custom_hp_hz);
  p.putFloat("lp", s.custom_lp_hz);
  p.putFloat("notch", s.notch_hz);
  p.putFloat("gain", s.gain);
  p.putUChar("order", s.order);
  p.putBool("ble", ble_enabled);
  p.end();
  return true;
}

void clear() {
  Preferences p;
  if (p.begin(kNamespace, false)) {
    p.clear();
    p.end();
  }
}

}  // namespace settings_store
