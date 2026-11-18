#include "hr_estimator.h"

#include <math.h>

namespace dsp {

namespace {
// Normalisation headroom: the running peak is scaled up slightly so that a beat
// a little louder than the previous ones is not clipped to x_n = 1 (where the
// Shannon energy would drop to zero).
constexpr float kNormHeadroom = 1.2f;
}  // namespace

void HrEstimator::begin(float fs_in) {
  seg_len_ = static_cast<int>(fs_in / kEnvRateHz + 0.5f);
  if (seg_len_ < 1) seg_len_ = 1;
  reset();
}

void HrEstimator::reset() {
  seg_count_ = 0;
  seg_sum_ = 0.0f;
  seg_max_ = 0.0f;
  for (int i = 0; i < kNormSegments; ++i) seg_maxes_[i] = 0.0f;
  seg_max_pos_ = 0;
  norm_ = 0.0f;
  inv_norm_ = 0.0f;
  for (int i = 0; i < kWindow; ++i) env_[i] = 0.0f;
  env_pos_ = 0;
  env_filled_ = 0;
  since_update_ = 0;
  last_env_ = 0.0f;
  hist_n_ = 0;
  hist_pos_ = 0;
  invalid_run_ = 0;
  res_ = HrResult();
}

void HrEstimator::endSegment() {
  last_env_ = seg_sum_ / static_cast<float>(seg_len_);
  env_[env_pos_] = last_env_;
  if (++env_pos_ >= kWindow) env_pos_ = 0;
  if (env_filled_ < kWindow) ++env_filled_;

  // Update the running normaliser with this segment's peak.
  seg_maxes_[seg_max_pos_] = seg_max_;
  if (++seg_max_pos_ >= kNormSegments) seg_max_pos_ = 0;
  float m = 0.0f;
  for (int i = 0; i < kNormSegments; ++i) {
    if (seg_maxes_[i] > m) m = seg_maxes_[i];
  }
  norm_ = m;
  inv_norm_ = m > 1e-9f ? 1.0f / (kNormHeadroom * m) : 0.0f;

  seg_count_ = 0;
  seg_sum_ = 0.0f;
  seg_max_ = 0.0f;

  if (++since_update_ >= kEnvRateHz && env_filled_ >= kWindow) {
    since_update_ = 0;
    estimate();
  }
}

void HrEstimator::estimate() {
  // Unroll the ring buffer (oldest first) and remove the mean.
  float mean = 0.0f;
  for (int i = 0; i < kWindow; ++i) {
    const float v = env_[(env_pos_ + i) % kWindow];
    scratch_[i] = v;
    mean += v;
  }
  mean /= kWindow;
  for (int i = 0; i < kWindow; ++i) scratch_[i] -= mean;

  auto acf = [this](int lag) {
    float s = 0.0f;
    for (int i = 0; i + lag < kWindow; ++i) s += scratch_[i] * scratch_[i + lag];
    return s;
  };

  ++res_.updates;
  const float r0 = acf(0);
  res_.pulsatility = mean > 1e-12f ? sqrtf(r0 / kWindow) / mean : 0.0f;
  if (r0 <= 1e-12f || res_.pulsatility < kMinPulsatility) {
    res_.confidence = 0.0f;
    reject();
    return;
  }

  const int lag_min = static_cast<int>(60.0f * kEnvRateHz / kMaxBpm);        // 15
  const int lag_max = static_cast<int>(60.0f * kEnvRateHz / kMinBpm + 0.5f); // 75

  // R(lag) for lag_min-1 .. lag_max+1 (neighbours needed for peak checks).
  float r[96];
  const int n_lags = lag_max - lag_min + 3;
  for (int j = 0; j < n_lags && j < 96; ++j) r[j] = acf(lag_min - 1 + j) / r0;

  int best = -1;
  float best_val = -1.0f;
  for (int j = 1; j < n_lags - 1; ++j) {
    if (r[j] >= r[j - 1] && r[j] >= r[j + 1] && r[j] > best_val) {
      best_val = r[j];
      best = j;
    }
  }
  if (best < 0) {
    res_.confidence = 0.0f;
    reject();
    return;
  }

  // Parabolic interpolation around the peak.
  const float ym1 = r[best - 1], y0 = r[best], yp1 = r[best + 1];
  const float denom = ym1 - 2.0f * y0 + yp1;
  float delta = 0.0f;
  if (fabsf(denom) > 1e-12f) delta = 0.5f * (ym1 - yp1) / denom;
  if (delta > 0.5f) delta = 0.5f;
  if (delta < -0.5f) delta = -0.5f;
  const float lag = static_cast<float>(lag_min - 1 + best) + delta;
  const float bpm = 60.0f * kEnvRateHz / lag;

  res_.bpm_instant = bpm;
  res_.confidence = best_val;
  if (best_val < kMinConfidence || bpm < kMinBpm || bpm > kMaxBpm) {
    reject();
    return;
  }
  invalid_run_ = 0;

  hist_[hist_pos_] = bpm;
  hist_pos_ = (hist_pos_ + 1) % kMedianLen;
  if (hist_n_ < kMedianLen) ++hist_n_;

  // Median of the stored estimates (insertion sort on a tiny array).
  float s[kMedianLen];
  for (int i = 0; i < hist_n_; ++i) s[i] = hist_[i];
  for (int i = 1; i < hist_n_; ++i) {
    const float v = s[i];
    int k = i - 1;
    while (k >= 0 && s[k] > v) {
      s[k + 1] = s[k];
      --k;
    }
    s[k + 1] = v;
  }
  res_.bpm = s[hist_n_ / 2];
  res_.valid = true;
}

void HrEstimator::reject() {
  res_.valid = false;
  // After a few seconds without a usable signal (stethoscope lifted, moved),
  // forget the old estimates so the median restarts from fresh data.
  if (++invalid_run_ >= 3) {
    hist_n_ = 0;
    hist_pos_ = 0;
  }
}

}  // namespace dsp
