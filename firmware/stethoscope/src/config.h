// Build-time configuration of the MEMS electronic stethoscope firmware.
//
// Hardware: Main board V1 (Altium sheet "MCU Application Circuit" + "Filter Circuit")
//   IC1 ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB octal PSRAM; IO35-37 used by the PSRAM)
//   IO4  (module pin 4)        <- AFE output (U4 buffer after the Sallen-Key LPF), biased at 1.65 V
//   IO19/IO20 (pins 13/14)     <- USB_D_N / USB_D_P from J2 (ESD D1, D2) -> USB-Serial/JTAG = "Serial"
//   RXD0/TXD0 (pins 36/37)     <- J1 UART header: 1 = 3V3, 2 = TX (ESP out), 3 = RX (ESP in), 4 = GND
//                                 -> optional link "Serial0" (LINK_USE_UART 1)
//   IO0  (pin 27, BOOT_GPIO0)  <- P1 jumper to GND: short while resetting = download mode
//   EN   (pin 3)               <- SW1 reset, R1 10k pull-up, C4 1 uF + C5 100 nF
//   No LEDs or buttons on other GPIOs.
//
// This header must stay free of Arduino / ESP-IDF includes: it is also used by
// the host-side unit tests in firmware/test/.
#pragma once

#include <stdint.h>

#define FW_NAME "MEMS-Stethoscope"
#define FW_VERSION "1.0.0"

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
#define PIN_AFE_OUT 4  // GPIO4 = ADC1 channel 3 (ADC1 keeps working while BLE/Wi-Fi run)

// ---------------------------------------------------------------------------
// Sampling and DSP
// ---------------------------------------------------------------------------
// The ADC oversamples at 16 kHz; a 63-tap FIR removes everything above 2 kHz and
// the stream is decimated by 4 to 4 kHz. Heart sounds lie in 20-600 Hz, so 4 kHz
// gives a wide margin above Nyquist (also matches the 64 kbps BLE budget in the
// design review: 4000 Hz x 16 bit).
#define ADC_SAMPLE_RATE_HZ 16000
#define DECIMATION 4
#define FS_OUT_HZ (ADC_SAMPLE_RATE_HZ / DECIMATION)  // 4000 Hz
#define ADC_FRAME_SAMPLES 256                         // DMA frame = 16 ms
#define BLOCK_SAMPLES (ADC_FRAME_SAMPLES / DECIMATION) // 64 output samples per block

#define ADC_BITS 12
#define ADC_MID_CODE 2048
// Codes this close to the rails are counted as clipping (AFE gain too high:
// turn trimmer R2 down).
#define ADC_CLIP_MARGIN 8

// Normalised sample: -1.0 .. +1.0 over the ADC range.
static inline float adc_code_to_float(uint16_t code) {
  return (static_cast<float>(code) - static_cast<float>(ADC_MID_CODE)) *
         (1.0f / static_cast<float>(ADC_MID_CODE));
}

// ---------------------------------------------------------------------------
// Default user settings (can be changed at runtime and saved to flash)
// ---------------------------------------------------------------------------
#define DEFAULT_MODE 1          // dsp::FilterMode::kHeart
#define DEFAULT_NOTCH_HZ 50.0f  // Sri Lanka mains = 50 Hz (use 60 in 60 Hz countries)
#define DEFAULT_GAIN 1.0f
#define DEFAULT_ORDER 4

// ---------------------------------------------------------------------------
// PC link: native USB CDC by default. LINK_USE_UART=1 uses UART0 instead (J1
// header on our board, or the "COM"/CH34x USB port of an ESP32-S3 DevKit).
// `make BOARD=devkit` sets it automatically.
// ---------------------------------------------------------------------------
#ifndef LINK_USE_UART
#define LINK_USE_UART 0
#endif
#define LINK_UART_BAUD 921600
#define SERIAL_TX_BUFFER 8192

// ---------------------------------------------------------------------------
// Bluetooth Low Energy streaming
// ---------------------------------------------------------------------------
#define BLE_ENABLED_DEFAULT 1
#define BLE_DEVICE_NAME "MEMS-Stetho"
#define BLE_SERVICE_UUID "6e5a0001-5354-4554-484f-53434f504531"
#define BLE_AUDIO_UUID "6e5a0002-5354-4554-484f-53434f504531"   // notify: int16 filtered @ 4 kHz
#define BLE_AUDIO_RF_UUID "6e5a0005-5354-4554-484f-53434f504531" // notify: int16 raw + filtered @ 4 kHz
#define BLE_STATUS_UUID "6e5a0003-5354-4554-484f-53434f504531"  // notify/read: JSON status
#define BLE_CONTROL_UUID "6e5a0004-5354-4554-484f-53434f504531" // write: text commands

// ---------------------------------------------------------------------------
// RTOS
// ---------------------------------------------------------------------------
#define DSP_TASK_CORE 1
#define DSP_TASK_PRIORITY 10
#define DSP_TASK_STACK 8192
#define BLE_TASK_CORE 0
#define BLE_TASK_PRIORITY 5
#define BLE_TASK_STACK 6144
#define AUDIO_QUEUE_LEN 32  // blocks (32 x 16 ms = 0.5 s of buffering)
#define BLE_QUEUE_LEN 16
