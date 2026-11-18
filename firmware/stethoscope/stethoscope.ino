// =============================================================================
//  Analog MEMS Electronic Stethoscope - ESP32-S3 firmware
//  BM4112 Medical Electronics & Instrumentation mini project
// =============================================================================
//
//  MEMS mic -> AFE (gain + 1.6 kHz anti-alias, 1.65 V bias) -> IO4 (ADC1_CH3)
//
//  ┌──────────── core 1 ─────────────┐      ┌──────────── core 0 ────────────┐
//  │ DSP task (prio 10)              │      │ BLE stack + BLE audio task      │
//  │  ADC DMA 16 kHz                 │ ───► │  notify filtered audio @ 4 kHz  │
//  │  FIR + decimate x4 -> 4 kHz     │      │  status JSON 1 Hz, commands     │
//  │  DC block -> notch -> BP filter │      └─────────────────────────────────┘
//  │  Shannon envelope -> HR         │
//  └───────────────┬─────────────────┘
//                  │ audio queue (64-sample blocks, 16 ms)
//  ┌───────────────▼─────────────────┐
//  │ Arduino loop (prio 1)           │  USB serial: commands, binary frames,
//  │                                 │  Serial-Plotter text, 1 Hz status
//  └─────────────────────────────────┘
//
//  Quick start: open the Serial Monitor (any baud, "Newline"), type "help".
//  PC software: firmware/tools (viewer.py / record.py / analyze.py).
// =============================================================================

#include <Arduino.h>
#include <math.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "src/app/app_state.h"
#include "src/app/buffer_print.h"
#include "src/app/commands.h"
#include "src/app/settings_store.h"
#include "src/config.h"
#include "src/dsp/fir_decim_coeffs.h"
#include "src/dsp/pcg_pipeline.h"
#include "src/hal/adc_sampler.h"
#include "src/link/ble_link.h"
#include "src/link/protocol.h"

// The ADC DMA format, GPIO4 = ADC1_CH3, two-core task layout and BLE all assume an ESP32-S3.
#if !CONFIG_IDF_TARGET_ESP32S3
#error "This firmware needs an ESP32-S3: select board 'ESP32S3 Dev Module' (or use: make upload BOARD=auto)"
#endif

#if LINK_USE_UART
#define LINK Serial0
#else
#define LINK Serial
#endif

struct AudioBlock {
  uint32_t first_index;
  uint16_t n;
  uint8_t mode;
  int16_t raw[BLOCK_SAMPLES];
  int16_t filt[BLOCK_SAMPLES];
};

static AdcSampler g_adc;
static dsp::PcgPipeline g_pipe;
static QueueHandle_t g_audio_q = nullptr;
static bool g_adc_ok = false;

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------
static inline int16_t toInt16(float v) {
  v *= 32767.0f;
  if (v > 32767.0f) return 32767;
  if (v < -32767.0f) return -32767;
  return static_cast<int16_t>(lrintf(v));
}

static inline float toDbfs(double mean_square) {
  return mean_square > 1e-14 ? 10.0f * log10f(static_cast<float>(mean_square)) : -140.0f;
}

static void sendFrame(uint8_t type, const uint8_t* payload, size_t len) {
  static uint8_t frame[1600];
  const size_t n = proto::encodeFrame(type, payload, len, frame, sizeof(frame));
  if (n) LINK.write(frame, n);
}

// ----------------------------------------------------------------------------
// DSP task: ADC -> pipeline -> blocks
// ----------------------------------------------------------------------------
static void dspTask(void*) {
  static uint16_t codes[ADC_FRAME_SAMPLES];
  static AudioBlock blk;
  uint16_t fill = 0;
  uint32_t out_index = 0;

  // One-second statistics
  int64_t t0 = esp_timer_get_time();
  uint32_t n_in = 0, n_out = 0, clips = 0;
  double sum_code = 0, sum_r2 = 0, sum_f2 = 0;
  float peak = 0;
  app::Telemetry tel;

  for (;;) {
    dsp::PipelineSettings s;
    if (app::takePendingSettings(&s)) g_pipe.configure(s);

    const size_t n = g_adc.read(codes, ADC_FRAME_SAMPLES, 100);
    for (size_t i = 0; i < n; ++i) {
      const uint16_t c = codes[i];
      sum_code += c;
      if (c <= ADC_CLIP_MARGIN || c >= (1 << ADC_BITS) - 1 - ADC_CLIP_MARGIN) ++clips;

      float r, f;
      if (!g_pipe.push(adc_code_to_float(c), &r, &f)) continue;

      sum_r2 += r * r;
      sum_f2 += f * f;
      const float ar = fabsf(r);
      if (ar > peak) peak = ar;
      ++n_out;

      if (fill == 0) {
        blk.first_index = out_index;
        blk.mode = static_cast<uint8_t>(g_pipe.settings().mode);
      }
      blk.raw[fill] = toInt16(r);
      blk.filt[fill] = toInt16(f);
      ++out_index;
      if (++fill == BLOCK_SAMPLES) {
        blk.n = fill;
        fill = 0;
        const app::StreamMode sm = app::streamMode();
        if (sm == app::StreamMode::kBinary || sm == app::StreamMode::kPlot) {
          if (xQueueSend(g_audio_q, &blk, 0) != pdTRUE) ++app::counters().serial_drops;
        }
        if (app::bleConnected()) ble_link::pushAudio(blk.first_index, blk.raw, blk.filt, blk.n);
      }
    }
    n_in += n;

    const int64_t now = esp_timer_get_time();
    if (now - t0 >= 1000000) {
      const double dt = (now - t0) * 1e-6;
      tel.fs_measured = static_cast<float>(n_in / dt);
      tel.rms_raw_dbfs = n_out ? toDbfs(sum_r2 / n_out) : -140.0f;
      tel.rms_filt_dbfs = n_out ? toDbfs(sum_f2 / n_out) : -140.0f;
      tel.peak_raw = peak;
      tel.dc_mv = n_in ? g_adc.codeToMillivolts(static_cast<int>(sum_code / n_in + 0.5)) : 0;
      tel.clip_last = clips;
      tel.clip_total += clips;
      tel.adc_overflows = g_adc.overflows();
      tel.sample_index = out_index;
      tel.hr = g_pipe.hr();
      app::publishTelemetry(tel);
      t0 = now;
      n_in = n_out = clips = 0;
      sum_code = sum_r2 = sum_f2 = 0;
      peak = 0;
    }
  }
}

// ----------------------------------------------------------------------------
// Serial link
// ----------------------------------------------------------------------------
static void runCommand(char* line) {
  static BufferPrint<1536> reply;  // replies are sent as text or as a TEXT frame
  reply.clear();
  const app::StreamMode before = app::streamMode();
  commands::execute(line, reply);
  if (!reply.length()) return;
  if (before == app::StreamMode::kBinary) {
    sendFrame(proto::kFrameText, reinterpret_cast<const uint8_t*>(reply.c_str()), reply.length());
  } else {
    LINK.write(reinterpret_cast<const uint8_t*>(reply.c_str()), reply.length());
  }
}

static void pollCommands() {
  static char line[128];
  static size_t len = 0;
  while (LINK.available()) {
    const int ch = LINK.read();
    if (ch == '\n' || ch == '\r') {
      if (len) {
        line[len] = '\0';
        runCommand(line);
        len = 0;
      }
    } else if (ch == 8 || ch == 127) {
      if (len) --len;
    } else if (len < sizeof(line) - 1 && ch >= 32) {
      line[len++] = static_cast<char>(ch);
    }
  }
}

static void drainAudio() {
  static AudioBlock blk;
  static uint8_t frame[BLOCK_SAMPLES * 4 + 32];
  static char text[BLOCK_SAMPLES / 8 * 40];
  while (xQueueReceive(g_audio_q, &blk, 0) == pdTRUE) {
    const app::StreamMode sm = app::streamMode();
    if (sm == app::StreamMode::kBinary) {
      const size_t n = proto::encodeAudioFrame(blk.first_index, blk.raw, blk.filt, blk.n, blk.mode,
                                               frame, sizeof(frame));
      if (LINK.availableForWrite() >= static_cast<int>(n)) {
        LINK.write(frame, n);
      } else {
        ++app::counters().serial_drops;
      }
    } else if (sm == app::StreamMode::kPlot) {
      // Every 8th sample -> 500 Hz, readable by the Arduino Serial Plotter.
      size_t len = 0;
      for (uint16_t i = 0; i < blk.n; i += 8) {
        len += snprintf(text + len, sizeof(text) - len, "raw:%d,filtered:%d\n", blk.raw[i],
                        blk.filt[i]);
      }
      if (LINK.availableForWrite() >= static_cast<int>(len)) LINK.write(text, len);
    }
  }
}

static void periodicStatus() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < 1000) return;
  last = now;

  static char buf[640];
  const size_t n = commands::statusJson(buf, sizeof(buf));
  switch (app::streamMode()) {
    case app::StreamMode::kBinary:
      sendFrame(proto::kFrameStatus, reinterpret_cast<const uint8_t*>(buf), n);
      break;
    case app::StreamMode::kLog: {
      static char line[256];
      const size_t m = commands::statusLine(line, sizeof(line));
      if (LINK.availableForWrite() >= static_cast<int>(m)) LINK.write(line, m);
      break;
    }
    default:
      break;
  }
  if (app::bleConnected()) ble_link::publishStatus(buf, n);
}

// ----------------------------------------------------------------------------
// Arduino entry points
// ----------------------------------------------------------------------------
void setup() {
#if LINK_USE_UART
  LINK.setTxBufferSize(SERIAL_TX_BUFFER);
  LINK.begin(LINK_UART_BAUD);
#else
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
  LINK.setTxBufferSize(SERIAL_TX_BUFFER);
#endif
  LINK.begin(115200);  // baud rate is ignored by USB CDC
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
  LINK.setTxTimeoutMs(0);  // never block the loop when no PC is attached
#endif
#endif
  delay(200);

  bool ble_on = BLE_ENABLED_DEFAULT;
  const dsp::PipelineSettings s = settings_store::load(&ble_on);
  app::init(s, ble_on);

  g_pipe.begin(ADC_SAMPLE_RATE_HZ, kFirDecimTaps, FIR_DECIM_NUM_TAPS, DECIMATION);
  g_pipe.configure(s);

  LINK.printf("\n%s v%s - ESP32-S3, AFE on GPIO%d, ADC %d Hz -> %d Hz\n", FW_NAME, FW_VERSION,
              PIN_AFE_OUT, ADC_SAMPLE_RATE_HZ, FS_OUT_HZ);

  g_audio_q = xQueueCreate(AUDIO_QUEUE_LEN, sizeof(AudioBlock));

  if (ble_on) {
    if (ble_link::begin()) {
      LINK.printf("BLE advertising as \"%s\"\n", BLE_DEVICE_NAME);
    } else {
      LINK.println("BLE init FAILED");
    }
  } else {
    LINK.println("BLE disabled (\"ble on\" + reboot to enable)");
  }

  g_adc_ok = g_adc.begin(PIN_AFE_OUT, ADC_SAMPLE_RATE_HZ, ADC_FRAME_SAMPLES);
  if (!g_adc_ok) {
    LINK.println("ERROR: ADC init failed");
  } else {
    LINK.printf("ADC calibration: %s\n", g_adc.calibrated() ? "eFuse curve fitting" : "nominal");
    xTaskCreatePinnedToCore(dspTask, "dsp", DSP_TASK_STACK, nullptr, DSP_TASK_PRIORITY, nullptr,
                            DSP_TASK_CORE);
  }
  LINK.println("Type 'help' for commands.");
}

void loop() {
  pollCommands();
  drainAudio();
  periodicStatus();
  if (commands::rebootRequested()) {
    LINK.flush();
    delay(200);
    ESP.restart();
  }
  delay(2);
}
