#include "biquad.h"

#include <math.h>

namespace dsp {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Coefficients are computed in double precision (design runs rarely, only when
// the user changes a setting) and stored as float for the real-time path.
BiquadCoeffs normalise(double b0, double b1, double b2, double a0, double a1,
                       double a2) {
  BiquadCoeffs c;
  c.b0 = static_cast<float>(b0 / a0);
  c.b1 = static_cast<float>(b1 / a0);
  c.b2 = static_cast<float>(b2 / a0);
  c.a1 = static_cast<float>(a1 / a0);
  c.a2 = static_cast<float>(a2 / a0);
  return c;
}

}  // namespace

BiquadCoeffs designLowpass(float fs, float f0, float q) {
  const double w0 = 2.0 * kPi * f0 / fs;
  const double cs = cos(w0), sn = sin(w0);
  const double alpha = sn / (2.0 * q);
  return normalise((1.0 - cs) / 2.0, 1.0 - cs, (1.0 - cs) / 2.0,
                   1.0 + alpha, -2.0 * cs, 1.0 - alpha);
}

BiquadCoeffs designHighpass(float fs, float f0, float q) {
  const double w0 = 2.0 * kPi * f0 / fs;
  const double cs = cos(w0), sn = sin(w0);
  const double alpha = sn / (2.0 * q);
  return normalise((1.0 + cs) / 2.0, -(1.0 + cs), (1.0 + cs) / 2.0,
                   1.0 + alpha, -2.0 * cs, 1.0 - alpha);
}

BiquadCoeffs designNotch(float fs, float f0, float q) {
  const double w0 = 2.0 * kPi * f0 / fs;
  const double cs = cos(w0), sn = sin(w0);
  const double alpha = sn / (2.0 * q);
  return normalise(1.0, -2.0 * cs, 1.0, 1.0 + alpha, -2.0 * cs, 1.0 - alpha);
}

float butterworthQ(int order, int k) {
  // Pole pair k of an N-th order Butterworth prototype sits at angle
  // theta_k = (2k+1)*pi/(2N) from the negative real axis; Q = 1/(2 cos theta).
  const double theta = (2.0 * k + 1.0) * kPi / (2.0 * order);
  return static_cast<float>(1.0 / (2.0 * cos(theta)));
}

void BiquadCascade::clear() {
  n_ = 0;
  reset();
}

void BiquadCascade::reset() {
  for (int i = 0; i < kMaxStages; ++i) z1_[i] = z2_[i] = 0.0f;
}

bool BiquadCascade::addStage(const BiquadCoeffs& c) {
  if (n_ >= kMaxStages) return false;
  c_[n_] = c;
  z1_[n_] = z2_[n_] = 0.0f;
  ++n_;
  return true;
}

bool BiquadCascade::addButterworthLowpass(float fs, float fc, int order) {
  if (order < 2) order = 2;
  for (int k = 0; k < order / 2; ++k) {
    if (!addStage(designLowpass(fs, fc, butterworthQ(order, k)))) return false;
  }
  return true;
}

bool BiquadCascade::addButterworthHighpass(float fs, float fc, int order) {
  if (order < 2) order = 2;
  for (int k = 0; k < order / 2; ++k) {
    if (!addStage(designHighpass(fs, fc, butterworthQ(order, k)))) return false;
  }
  return true;
}

bool BiquadCascade::addNotch(float fs, float f0, float q) {
  return addStage(designNotch(fs, f0, q));
}

}  // namespace dsp
