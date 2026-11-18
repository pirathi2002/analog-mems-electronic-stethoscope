#include "adc_sampler.h"

#include <stdlib.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char* TAG = "adc";

// 12 dB attenuation: input range ~0..3.1 V, enough for the AFE swing around
// the 1.65 V bias.
static constexpr adc_atten_t kAtten = ADC_ATTEN_DB_12;

struct AdcSamplerIsr {
  static bool IRAM_ATTR onOverflow(adc_continuous_handle_t, const adc_continuous_evt_data_t*,
                                   void* user) {
    static_cast<AdcSampler*>(user)->overflows_ = static_cast<AdcSampler*>(user)->overflows_ + 1;
    return false;
  }
};

bool AdcSampler::begin(int gpio, uint32_t sample_rate_hz, uint32_t frame_samples) {
  adc_unit_t unit;
  adc_channel_t channel;
  if (adc_continuous_io_to_channel(gpio, &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
    ESP_LOGE(TAG, "GPIO%d is not an ADC1 pin", gpio);
    return false;
  }
  unit_ = unit;
  channel_ = channel;

  buf_len_ = frame_samples * SOC_ADC_DIGI_RESULT_BYTES;
  buf_ = static_cast<uint8_t*>(malloc(buf_len_));
  if (!buf_) return false;

  adc_continuous_handle_t h = nullptr;
  adc_continuous_handle_cfg_t hcfg = {};
  hcfg.max_store_buf_size = buf_len_ * 8;  // 8 frames = 128 ms of slack
  hcfg.conv_frame_size = buf_len_;
  if (adc_continuous_new_handle(&hcfg, &h) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_new_handle failed");
    return false;
  }

  adc_digi_pattern_config_t pattern = {};
  pattern.atten = kAtten;
  pattern.channel = channel & 0x7;
  pattern.unit = unit;
  pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t cfg = {};
  cfg.pattern_num = 1;
  cfg.adc_pattern = &pattern;
  cfg.sample_freq_hz = sample_rate_hz;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_continuous_config(h, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_config failed");
    adc_continuous_deinit(h);
    return false;
  }

  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = AdcSamplerIsr::onOverflow;
  adc_continuous_register_event_callbacks(h, &cbs, this);

  // Calibration (curve fitting on ESP32-S3) for the DC-bias diagnostic.
  adc_cali_handle_t cali = nullptr;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t ccfg = {};
  ccfg.unit_id = unit;
  ccfg.chan = channel;
  ccfg.atten = kAtten;
  ccfg.bitwidth = ADC_BITWIDTH_12;
  if (adc_cali_create_scheme_curve_fitting(&ccfg, &cali) != ESP_OK) cali = nullptr;
#endif
  cali_ = cali;

  if (adc_continuous_start(h) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_start failed");
    adc_continuous_deinit(h);
    return false;
  }
  handle_ = h;
  return true;
}

void AdcSampler::end() {
  if (handle_) {
    adc_continuous_stop(static_cast<adc_continuous_handle_t>(handle_));
    adc_continuous_deinit(static_cast<adc_continuous_handle_t>(handle_));
    handle_ = nullptr;
  }
  free(buf_);
  buf_ = nullptr;
}

size_t AdcSampler::read(uint16_t* out, size_t max_samples, uint32_t timeout_ms) {
  if (!handle_) return 0;
  uint32_t got = 0;
  size_t want = max_samples * SOC_ADC_DIGI_RESULT_BYTES;
  if (want > buf_len_) want = buf_len_;
  if (adc_continuous_read(static_cast<adc_continuous_handle_t>(handle_), buf_, want, &got,
                          timeout_ms) != ESP_OK) {
    return 0;
  }
  size_t n = 0;
  for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= got; i += SOC_ADC_DIGI_RESULT_BYTES) {
    const adc_digi_output_data_t* p = reinterpret_cast<const adc_digi_output_data_t*>(&buf_[i]);
    if (p->type2.unit != 0 || p->type2.channel != static_cast<uint32_t>(channel_)) continue;
    out[n++] = static_cast<uint16_t>(p->type2.data);
    if (n >= max_samples) break;
  }
  return n;
}

int AdcSampler::codeToMillivolts(int code) const {
  if (cali_) {
    int mv = 0;
    if (adc_cali_raw_to_voltage(static_cast<adc_cali_handle_t>(cali_), code, &mv) == ESP_OK) return mv;
  }
  return code * 3100 / 4095;  // nominal range at 12 dB attenuation
}
