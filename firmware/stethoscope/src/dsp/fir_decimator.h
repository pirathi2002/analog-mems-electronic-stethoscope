// FIR low-pass + decimate-by-M (anti-aliasing for the 16 kHz -> 4 kHz step).
//
// Output sample k corresponds to input sample k*M + (M-1), i.e. it matches
// scipy.signal.lfilter(h, 1, x)[M-1::M].
#pragma once

namespace dsp {

class FirDecimator {
 public:
  static constexpr int kMaxTaps = 128;

  // taps must stay valid for the lifetime of the decimator (e.g. a const table).
  void init(const float* taps, int ntaps, int factor) {
    h_ = taps;
    n_ = ntaps > kMaxTaps ? kMaxTaps : ntaps;
    m_ = factor < 1 ? 1 : factor;
    reset();
  }

  void reset() {
    for (int i = 0; i < 2 * kMaxTaps; ++i) buf_[i] = 0.0f;
    pos_ = 0;
    phase_ = 0;
  }

  int factor() const { return m_; }
  int taps() const { return n_; }

  // Push one input sample. Returns true and writes *out when an output sample
  // is produced (every M-th input).
  inline bool push(float x, float* out) {
    // Double-length circular buffer: each sample is written twice so that the
    // newest-to-oldest window is always contiguous at &buf_[pos_].
    if (--pos_ < 0) pos_ = n_ - 1;
    buf_[pos_] = x;
    buf_[pos_ + n_] = x;
    if (++phase_ < m_) return false;
    phase_ = 0;
    const float* p = &buf_[pos_];
    float acc = 0.0f;
    for (int k = 0; k < n_; ++k) acc += h_[k] * p[k];
    *out = acc;
    return true;
  }

 private:
  const float* h_ = nullptr;
  int n_ = 0;
  int m_ = 1;
  int pos_ = 0;
  int phase_ = 0;
  float buf_[2 * kMaxTaps];
};

}  // namespace dsp
