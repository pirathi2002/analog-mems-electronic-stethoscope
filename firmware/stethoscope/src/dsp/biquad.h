// Biquad (second-order IIR) filters and cascades.
//
// Portable C++ (no Arduino / ESP-IDF dependencies) so the exact same code can
// be compiled on a PC and verified against the SciPy reference in
// firmware/tools/stetho/dsp.py (see firmware/test/).
//
// Filter design follows the RBJ "Audio EQ Cookbook" bilinear-transform
// formulas with frequency pre-warping. Butterworth filters of even order N are
// built as N/2 cascaded biquads with the classic Butterworth pole Q values, which
// gives exactly the same response as scipy.signal.butter(N, fc, fs=fs).
#pragma once

#include <stddef.h>

namespace dsp {

struct BiquadCoeffs {
  // Normalised so that a0 == 1:
  //   H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a1 = 0.0f, a2 = 0.0f;
};

BiquadCoeffs designLowpass(float fs, float f0, float q);
BiquadCoeffs designHighpass(float fs, float f0, float q);
BiquadCoeffs designNotch(float fs, float f0, float q);

// Q of biquad section k (0-based) of an N-th order Butterworth filter (N even).
float butterworthQ(int order, int k);

class BiquadCascade {
 public:
  static constexpr int kMaxStages = 8;

  void clear();    // remove all stages (cascade becomes a pass-through)
  void reset();    // zero the filter state, keep coefficients
  bool addStage(const BiquadCoeffs& c);

  // order must be even (2, 4, 6, 8). Returns false if out of stages.
  bool addButterworthLowpass(float fs, float fc, int order);
  bool addButterworthHighpass(float fs, float fc, int order);
  bool addNotch(float fs, float f0, float q);

  int stages() const { return n_; }
  const BiquadCoeffs& coeffs(int i) const { return c_[i]; }

  // Transposed Direct Form II: numerically well behaved in float32.
  inline float process(float x) {
    for (int i = 0; i < n_; ++i) {
      const BiquadCoeffs& c = c_[i];
      const float y = c.b0 * x + z1_[i];
      z1_[i] = c.b1 * x - c.a1 * y + z2_[i];
      z2_[i] = c.b2 * x - c.a2 * y;
      x = y;
    }
    return x;
  }

 private:
  BiquadCoeffs c_[kMaxStages];
  float z1_[kMaxStages] = {0};
  float z2_[kMaxStages] = {0};
  int n_ = 0;
};

}  // namespace dsp
