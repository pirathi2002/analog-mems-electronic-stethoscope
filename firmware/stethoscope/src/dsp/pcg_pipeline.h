// Complete PCG (phonocardiogram) digital signal chain.
//
//   ADC 16 kHz ──► FIR anti-alias + decimate x4 ──► DC blocker ──► "raw" (4 kHz)
//                                                      │
//                       ┌──────────────────────────────┤
//                       ▼                              ▼
//          mains notch (50/60 Hz)          HR branch: notch ─► BP 25-150 Hz
//                       ▼                              ▼
//      Butterworth HP ─► Butterworth LP          Shannon envelope ─► HR
//         (band set by the filter mode)
//                       ▼
//                 digital gain ──► "filtered" (4 kHz)
//
// The HR branch always uses its own fixed band, so heart rate keeps working
// regardless of which listening mode the user selects.
#pragma once

#include <stdint.h>

#include "biquad.h"
#include "fir_decimator.h"
#include "hr_estimator.h"

namespace dsp {

enum class FilterMode : uint8_t {
  kRaw = 0,        // no digital filtering (DC removed only) - reference for comparisons
  kHeart = 1,      // 25-400 Hz  : standard PCG band (S1, S2, most murmurs)
  kBell = 2,       // 20-200 Hz  : low-frequency sounds (S3, S4, low-pitched murmurs)
  kDiaphragm = 3,  // 100-500 Hz : high-pitched sounds (lung sounds, high murmurs)
  kWide = 4,       // 20-1000 Hz : extended range
  kCustom = 5,     // user-defined high-pass / low-pass corners
};

constexpr int kNumFilterModes = 6;

struct PipelineSettings {
  FilterMode mode = FilterMode::kHeart;
  float custom_hp_hz = 25.0f;
  float custom_lp_hz = 400.0f;
  float notch_hz = 50.0f;   // 0 = notch disabled (Sri Lanka mains = 50 Hz)
  float gain = 1.0f;        // linear digital gain applied to the filtered output
  uint8_t order = 4;        // Butterworth order of the HP and LP sections (2/4/6)
};

const char* modeName(FilterMode m);
bool parseMode(const char* name, FilterMode* out);
// Corner frequencies for a mode (hp_hz == 0 or lp_hz == 0 means "section off").
void modeBand(const PipelineSettings& s, float* hp_hz, float* lp_hz);

class PcgPipeline {
 public:
  static constexpr float kHrBandLowHz = 25.0f;
  static constexpr float kHrBandHighHz = 150.0f;
  static constexpr float kNotchQ = 30.0f;
  static constexpr float kDcBlockHz = 0.5f;

  // fs_in: ADC sample rate; taps/ntaps: decimation FIR; decim: decimation factor.
  void begin(float fs_in, const float* taps, int ntaps, int decim);
  void configure(const PipelineSettings& s);  // re-designs the user filter chain
  void reset();                               // clears all filter states

  float fsOut() const { return fs_out_; }
  const PipelineSettings& settings() const { return settings_; }
  const HrResult& hr() const { return hr_.result(); }

  // Push one ADC sample (normalised: -1..+1 full scale). Returns true when an
  // output pair (raw, filtered) at fs_out is produced.
  inline bool push(float x, float* raw, float* filtered) {
    float d;
    if (!decim_.push(x, &d)) return false;

    // First-order DC blocker: y[n] = x[n] - x[n-1] + R*y[n-1]
    const float r = d - dc_x1_ + dc_r_ * dc_y1_;
    dc_x1_ = d;
    dc_y1_ = r;
    *raw = r;

    // Heart-rate branch (fixed band)
    hr_.push(hr_band_.process(hr_notch_.process(r)));

    // User listening chain
    float y = r;
    if (settings_.mode != FilterMode::kRaw) {
      y = notch_.process(y);
      y = band_.process(y);
    }
    *filtered = y * settings_.gain;
    return true;
  }

 private:
  float fs_in_ = 16000.0f;
  float fs_out_ = 4000.0f;
  FirDecimator decim_;
  float dc_r_ = 0.999f;
  float dc_x1_ = 0.0f;
  float dc_y1_ = 0.0f;

  PipelineSettings settings_;
  BiquadCascade notch_;
  BiquadCascade band_;

  BiquadCascade hr_notch_;
  BiquadCascade hr_band_;
  HrEstimator hr_;
};

}  // namespace dsp
