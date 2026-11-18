#include "pcg_pipeline.h"

#include <math.h>
#include <string.h>

namespace dsp {

namespace {

struct ModeInfo {
  const char* name;
  float hp_hz;
  float lp_hz;
};

// Band edges follow common electronic-stethoscope practice (bell / diaphragm /
// extended modes) and the PCG literature (25-400 Hz pre-processing band).
constexpr ModeInfo kModes[kNumFilterModes] = {
    {"raw", 0.0f, 0.0f},
    {"heart", 25.0f, 400.0f},
    {"bell", 20.0f, 200.0f},
    {"diaphragm", 100.0f, 500.0f},
    {"wide", 20.0f, 1000.0f},
    {"custom", 0.0f, 0.0f},
};

}  // namespace

const char* modeName(FilterMode m) {
  const int i = static_cast<int>(m);
  return (i >= 0 && i < kNumFilterModes) ? kModes[i].name : "?";
}

bool parseMode(const char* name, FilterMode* out) {
  for (int i = 0; i < kNumFilterModes; ++i) {
    if (strcmp(name, kModes[i].name) == 0) {
      *out = static_cast<FilterMode>(i);
      return true;
    }
  }
  return false;
}

void modeBand(const PipelineSettings& s, float* hp_hz, float* lp_hz) {
  if (s.mode == FilterMode::kCustom) {
    *hp_hz = s.custom_hp_hz;
    *lp_hz = s.custom_lp_hz;
  } else {
    const int i = static_cast<int>(s.mode);
    *hp_hz = kModes[i].hp_hz;
    *lp_hz = kModes[i].lp_hz;
  }
}

void PcgPipeline::begin(float fs_in, const float* taps, int ntaps, int decim) {
  fs_in_ = fs_in;
  fs_out_ = fs_in / decim;
  decim_.init(taps, ntaps, decim);
  dc_r_ = 1.0f - 2.0f * 3.14159265f * kDcBlockHz / fs_out_;

  hr_notch_.clear();
  hr_band_.clear();
  hr_band_.addButterworthHighpass(fs_out_, kHrBandLowHz, 4);
  hr_band_.addButterworthLowpass(fs_out_, kHrBandHighHz, 4);
  hr_.begin(fs_out_);

  configure(settings_);
  reset();
}

void PcgPipeline::configure(const PipelineSettings& in) {
  PipelineSettings s = in;
  const float nyq = 0.5f * fs_out_;
  if (s.order != 2 && s.order != 4 && s.order != 6) s.order = 4;
  if (!(s.gain > 0.0f)) s.gain = 1.0f;
  if (s.gain > 1000.0f) s.gain = 1000.0f;

  float hp, lp;
  modeBand(s, &hp, &lp);
  // Sanitise corners: anything outside (0, 0.45*fs) disables that section.
  if (!(hp > 0.0f) || hp >= 0.45f * fs_out_) hp = 0.0f;
  if (!(lp > 0.0f) || lp >= 0.45f * fs_out_) lp = 0.0f;
  if (hp > 0.0f && lp > 0.0f && hp >= lp) lp = 0.0f;

  notch_.clear();
  if (s.notch_hz > 0.0f && s.notch_hz < nyq) {
    notch_.addNotch(fs_out_, s.notch_hz, kNotchQ);
  } else {
    s.notch_hz = 0.0f;
  }

  band_.clear();
  if (hp > 0.0f) band_.addButterworthHighpass(fs_out_, hp, s.order);
  if (lp > 0.0f) band_.addButterworthLowpass(fs_out_, lp, s.order);

  // HR branch follows the mains notch setting.
  hr_notch_.clear();
  if (s.notch_hz > 0.0f) hr_notch_.addNotch(fs_out_, s.notch_hz, kNotchQ);

  settings_ = s;
}

void PcgPipeline::reset() {
  decim_.reset();
  dc_x1_ = dc_y1_ = 0.0f;
  notch_.reset();
  band_.reset();
  hr_notch_.reset();
  hr_band_.reset();
  hr_.reset();
}

}  // namespace dsp
