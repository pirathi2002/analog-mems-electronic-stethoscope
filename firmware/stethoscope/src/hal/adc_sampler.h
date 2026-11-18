// Continuous (DMA) ADC sampling of the AFE output on the ESP32-S3.
//
// Uses the ESP-IDF adc_continuous driver directly (not analogRead) so samples
// are taken by hardware at an exact rate with no CPU jitter, and are delivered
// in frames to the DSP task.
#pragma once

#include <stddef.h>
#include <stdint.h>

class AdcSampler {
 public:
  // gpio must be an ADC1 pin. Returns false on driver error.
  bool begin(int gpio, uint32_t sample_rate_hz, uint32_t frame_samples);
  void end();

  // Blocks up to timeout_ms for a frame; writes up to max_samples 12-bit codes.
  // Returns the number of samples written (0 on timeout).
  size_t read(uint16_t* out, size_t max_samples, uint32_t timeout_ms);

  // ADC code -> millivolts at the pin, using eFuse calibration if available.
  int codeToMillivolts(int code) const;
  bool calibrated() const { return cali_ != nullptr; }

  uint32_t overflows() const { return overflows_; }

 private:
  void* handle_ = nullptr;  // adc_continuous_handle_t
  void* cali_ = nullptr;    // adc_cali_handle_t
  int unit_ = 0;
  int channel_ = 0;
  uint8_t* buf_ = nullptr;
  size_t buf_len_ = 0;
  volatile uint32_t overflows_ = 0;

  friend struct AdcSamplerIsr;
};
