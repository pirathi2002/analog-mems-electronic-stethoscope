// Persists user settings in NVS flash (survive power cycles).
#pragma once

#include "../dsp/pcg_pipeline.h"

namespace settings_store {

dsp::PipelineSettings defaults();
// Loads saved settings (or defaults). ble_enabled receives the saved BLE flag.
dsp::PipelineSettings load(bool* ble_enabled);
bool save(const dsp::PipelineSettings& s, bool ble_enabled);
void clear();

}  // namespace settings_store
