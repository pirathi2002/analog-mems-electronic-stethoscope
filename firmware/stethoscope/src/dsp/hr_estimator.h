// Real-time heart-rate estimation from a band-passed PCG signal.
//
// Algorithm (same as estimate_hr_autocorr() in firmware/tools/stetho/dsp.py):
//   1. Normalised average Shannon energy envelope:
//        x_n = |y| / norm   (norm = running max |y| over the last 3 s)
//        E   = -x_n^2 * ln(x_n^2)
//      averaged over 20 ms segments -> envelope sampled at 50 Hz.
//      Shannon energy emphasises medium-intensity components (S1/S2) and
//      suppresses both low-level noise and isolated high-amplitude spikes.
//   2. Every second, the biased autocorrelation of the last 8 s of envelope is
//      computed. The cardiac period is the lag of the highest autocorrelation
//      peak between 60/max_bpm and 60/min_bpm seconds (parabolic
//      interpolation gives sub-sample accuracy).
//   3. The window is rejected if the envelope is not impulsive enough
//      (std/mean < kMinPulsatility): no heart sounds, only steady noise.
//   4. HR = 60 / period. The normalised peak height R(T)/R(0) is reported as a
//      confidence value; the output is the median of the last 5 valid
//      estimates.
#pragma once

#include <math.h>
#include <stdint.h>

namespace dsp {

struct HrResult {
  float bpm = 0.0f;         // median-smoothed heart rate
  float bpm_instant = 0.0f; // latest single-window estimate
  float confidence = 0.0f;  // autocorrelation peak height, 0..1
  float pulsatility = 0.0f; // envelope std/mean: heart sounds ~1.3-3, steady noise < 0.4
  bool valid = false;
  uint32_t updates = 0;     // number of estimates made so far
};

class HrEstimator {
 public:
  static constexpr int kEnvRateHz = 50;
  static constexpr int kWindowSec = 8;
  static constexpr int kWindow = kEnvRateHz * kWindowSec;  // envelope samples
  static constexpr int kNormSegments = 3 * kEnvRateHz;    // 3 s
  static constexpr int kMedianLen = 5;
  static constexpr float kMinBpm = 40.0f;
  static constexpr float kMaxBpm = 200.0f;
  static constexpr float kMinConfidence = 0.15f;
  // Heart sounds are impulsive (short S1/S2 bursts, quiet in between), so their
  // envelope varies strongly. Stationary noise (mains hum, hiss) gives an almost
  // flat envelope whose small ripples (e.g. beating of close mains harmonics)
  // could otherwise look periodic to the autocorrelation.
  static constexpr float kMinPulsatility = 0.8f;

  void begin(float fs_in);
  void reset();

  // Push one sample of the HR band-passed signal (at fs_in).
  inline void push(float y) {
    const float a = y < 0.0f ? -y : y;
    if (a > seg_max_) seg_max_ = a;
    if (norm_ > 1e-9f) {
      float xn = a * inv_norm_;
      if (xn > 1.0f) xn = 1.0f;
      const float e = xn * xn;
      if (e > 1e-12f) seg_sum_ += -e * logf(e);
    }
    if (++seg_count_ >= seg_len_) endSegment();
  }

  const HrResult& result() const { return res_; }

  // Most recent envelope value (useful for plotting/debug).
  float lastEnvelope() const { return last_env_; }

 private:
  void endSegment();
  void estimate();
  void reject();

  int seg_len_ = 80;
  int seg_count_ = 0;
  float seg_sum_ = 0.0f;
  float seg_max_ = 0.0f;

  float seg_maxes_[kNormSegments] = {0};
  int seg_max_pos_ = 0;
  float norm_ = 0.0f;
  float inv_norm_ = 0.0f;

  float env_[kWindow] = {0};
  int env_pos_ = 0;
  int env_filled_ = 0;
  int since_update_ = 0;
  float last_env_ = 0.0f;

  float scratch_[kWindow] = {0};
  float hist_[kMedianLen] = {0};
  int hist_n_ = 0;
  int hist_pos_ = 0;
  int invalid_run_ = 0;  // consecutive rejected windows

  HrResult res_;
};

}  // namespace dsp
