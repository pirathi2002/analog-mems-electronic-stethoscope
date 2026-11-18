#include "commands.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../config.h"
#include "app_state.h"
#include "settings_store.h"

namespace commands {

namespace {

volatile bool g_reboot = false;

void lower(char* s) {
  for (; *s; ++s) *s = static_cast<char>(tolower(static_cast<unsigned char>(*s)));
}

bool parseFloat(const char* s, float* out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  const float v = strtof(s, &end);
  if (end == s || *end != '\0') return false;
  *out = v;
  return true;
}

void printSettings(Print& out, const dsp::PipelineSettings& s) {
  float hp, lp;
  dsp::modeBand(s, &hp, &lp);
  out.printf("mode=%s band=%.0f-%.0f Hz order=%u notch=%s", dsp::modeName(s.mode), hp, lp,
             s.order, s.notch_hz > 0 ? "" : "off");
  if (s.notch_hz > 0) out.printf("%.0f Hz", s.notch_hz);
  out.printf(" gain=%.2f\n", s.gain);
}

}  // namespace

bool rebootRequested() { return g_reboot; }

void printHelp(Print& out) {
  out.print(
      "Commands:\n"
      "  help                      this list\n"
      "  status                    status as JSON\n"
      "  info                      firmware / hardware info\n"
      "  mode <m>                  raw | heart | bell | diaphragm | wide | custom\n"
      "                              heart 25-400 Hz, bell 20-200 Hz,\n"
      "                              diaphragm 100-500 Hz, wide 20-1000 Hz\n"
      "  band <hp_hz> <lp_hz>      custom band (sets mode custom; 0 = section off)\n"
      "  order <2|4|6>             Butterworth order of HP/LP sections\n"
      "  notch <off|50|60|hz>      mains notch filter\n"
      "  gain <x>                  digital gain of the filtered output (0.1-100)\n"
      "  stream <off|log|plot|bin> serial output: quiet, 1 line/s, Serial Plotter, binary\n"
      "  start | stop              same as 'stream bin' | 'stream off'\n"
      "  ble <on|off>              enable BLE streaming (saved, applies after reboot)\n"
      "  save                      store current settings in flash\n"
      "  defaults                  restore factory settings (not saved until 'save')\n"
      "  reboot                    restart the device\n");
}

size_t statusJson(char* buf, size_t cap) {
  const app::Telemetry t = app::telemetry();
  const dsp::PipelineSettings s = app::settings();
  float hp, lp;
  dsp::modeBand(s, &hp, &lp);
  const app::Counters& c = app::counters();
  const int n = snprintf(
      buf, cap,
      "{\"t\":\"status\",\"fw\":\"%s\",\"up\":%lu,\"fs_adc\":%.1f,\"fs\":%d,"
      "\"mode\":\"%s\",\"hp\":%.1f,\"lp\":%.1f,\"order\":%u,\"notch\":%.1f,\"gain\":%.3f,"
      "\"hr\":%.1f,\"hr_inst\":%.1f,\"hr_conf\":%.2f,\"hr_puls\":%.2f,\"hr_ok\":%d,"
      "\"rms\":%.1f,\"rms_f\":%.1f,\"peak\":%.3f,\"dc_mv\":%d,\"clip\":%lu,\"clip_total\":%lu,"
      "\"ovf\":%lu,\"drops\":%lu,\"ble_drops\":%lu,\"ble\":%d,\"ble_conn\":%d,"
      "\"stream\":\"%s\",\"idx\":%lu}",
      FW_VERSION, static_cast<unsigned long>(millis() / 1000), t.fs_measured, FS_OUT_HZ,
      dsp::modeName(s.mode), hp, lp, s.order, s.notch_hz, s.gain, t.hr.bpm, t.hr.bpm_instant,
      t.hr.confidence, t.hr.pulsatility, t.hr.valid ? 1 : 0, t.rms_raw_dbfs, t.rms_filt_dbfs, t.peak_raw, t.dc_mv,
      static_cast<unsigned long>(t.clip_last), static_cast<unsigned long>(t.clip_total),
      static_cast<unsigned long>(t.adc_overflows), static_cast<unsigned long>(c.serial_drops.load()),
      static_cast<unsigned long>(c.ble_drops.load()), app::bleEnabled() ? 1 : 0,
      app::bleConnected() ? 1 : 0, app::streamModeName(app::streamMode()),
      static_cast<unsigned long>(t.sample_index));
  return n < 0 ? 0 : (static_cast<size_t>(n) < cap ? n : cap - 1);
}

size_t statusLine(char* buf, size_t cap) {
  const app::Telemetry t = app::telemetry();
  const dsp::PipelineSettings s = app::settings();
  char hr[24];
  if (t.hr.valid) {
    snprintf(hr, sizeof(hr), "%5.1f bpm", t.hr.bpm);
  } else {
    snprintf(hr, sizeof(hr), "  --- bpm");
  }
  const int n = snprintf(buf, cap,
                         "HR %s (conf %.2f) | level %6.1f dBFS raw, %6.1f dBFS filt | "
                         "bias %4d mV | clip %lu | fs %.0f Hz | %s%s\n",
                         hr, t.hr.confidence, t.rms_raw_dbfs, t.rms_filt_dbfs, t.dc_mv,
                         static_cast<unsigned long>(t.clip_last), t.fs_measured,
                         dsp::modeName(s.mode), app::bleConnected() ? " | BLE connected" : "");
  return n < 0 ? 0 : (static_cast<size_t>(n) < cap ? n : cap - 1);
}

void execute(char* line, Print& out) {
  // Trim and tokenise.
  char* save = nullptr;
  lower(line);
  char* cmd = strtok_r(line, " \t\r\n", &save);
  if (!cmd) return;
  char* a1 = strtok_r(nullptr, " \t\r\n", &save);
  char* a2 = strtok_r(nullptr, " \t\r\n", &save);

  dsp::PipelineSettings s = app::settings();

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp(out);
  } else if (!strcmp(cmd, "status")) {
    char buf[640];
    statusJson(buf, sizeof(buf));
    out.println(buf);
  } else if (!strcmp(cmd, "info")) {
    out.printf("%s v%s  (built %s %s)\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);
    out.printf("MCU %s rev %d, %d cores @ %lu MHz, flash %lu MB, PSRAM %lu KB\n", ESP.getChipModel(),
               ESP.getChipRevision(), ESP.getChipCores(),
               static_cast<unsigned long>(ESP.getCpuFreqMHz()),
               static_cast<unsigned long>(ESP.getFlashChipSize() >> 20),
               static_cast<unsigned long>(ESP.getPsramSize() >> 10));
    out.printf("AFE input GPIO%d, ADC %d Hz -> decimate x%d -> %d Hz\n", PIN_AFE_OUT,
               ADC_SAMPLE_RATE_HZ, DECIMATION, FS_OUT_HZ);
    printSettings(out, s);
  } else if (!strcmp(cmd, "mode")) {
    dsp::FilterMode m;
    if (a1 && dsp::parseMode(a1, &m)) {
      s.mode = m;
      app::requestSettings(s);
      printSettings(out, s);
    } else {
      out.println("ERR usage: mode raw|heart|bell|diaphragm|wide|custom");
    }
  } else if (!strcmp(cmd, "band")) {
    float hp, lp;
    if (parseFloat(a1, &hp) && parseFloat(a2, &lp) && hp >= 0 && lp >= 0 && hp < 1800 &&
        lp < 1800 && (hp == 0 || lp == 0 || hp < lp)) {
      s.mode = dsp::FilterMode::kCustom;
      s.custom_hp_hz = hp;
      s.custom_lp_hz = lp;
      app::requestSettings(s);
      printSettings(out, s);
    } else {
      out.println("ERR usage: band <hp_hz> <lp_hz>  (0..1800 Hz, hp < lp, 0 = off)");
    }
  } else if (!strcmp(cmd, "order")) {
    const int o = a1 ? atoi(a1) : 0;
    if (o == 2 || o == 4 || o == 6) {
      s.order = static_cast<uint8_t>(o);
      app::requestSettings(s);
      printSettings(out, s);
    } else {
      out.println("ERR usage: order 2|4|6");
    }
  } else if (!strcmp(cmd, "notch")) {
    float f;
    if (a1 && !strcmp(a1, "off")) {
      s.notch_hz = 0;
    } else if (parseFloat(a1, &f) && f >= 0 && f < 1000) {
      s.notch_hz = f;
    } else {
      out.println("ERR usage: notch off|50|60|<hz>");
      return;
    }
    app::requestSettings(s);
    printSettings(out, s);
  } else if (!strcmp(cmd, "gain")) {
    float g;
    if (parseFloat(a1, &g) && g >= 0.1f && g <= 100.0f) {
      s.gain = g;
      app::requestSettings(s);
      printSettings(out, s);
    } else {
      out.println("ERR usage: gain <0.1..100>");
    }
  } else if (!strcmp(cmd, "stream") || !strcmp(cmd, "start") || !strcmp(cmd, "stop")) {
    const char* m = !strcmp(cmd, "start") ? "bin" : !strcmp(cmd, "stop") ? "off" : a1;
    app::StreamMode sm;
    if (m && !strcmp(m, "off")) {
      sm = app::StreamMode::kOff;
    } else if (m && !strcmp(m, "log")) {
      sm = app::StreamMode::kLog;
    } else if (m && !strcmp(m, "plot")) {
      sm = app::StreamMode::kPlot;
    } else if (m && (!strcmp(m, "bin") || !strcmp(m, "binary"))) {
      sm = app::StreamMode::kBinary;
    } else {
      out.println("ERR usage: stream off|log|plot|bin");
      return;
    }
    // Reply before switching so the acknowledgement is still readable text.
    out.printf("OK stream %s\n", app::streamModeName(sm));
    app::setStreamMode(sm);
  } else if (!strcmp(cmd, "ble")) {
    if (a1 && (!strcmp(a1, "on") || !strcmp(a1, "off"))) {
      app::setBleEnabled(!strcmp(a1, "on"));
      settings_store::save(s, app::bleEnabled());
      out.printf("OK ble %s (saved; reboot to apply)\n", a1);
    } else {
      out.println("ERR usage: ble on|off");
    }
  } else if (!strcmp(cmd, "save")) {
    out.println(settings_store::save(s, app::bleEnabled()) ? "OK saved" : "ERR save failed");
  } else if (!strcmp(cmd, "defaults")) {
    s = settings_store::defaults();
    app::requestSettings(s);
    printSettings(out, s);
  } else if (!strcmp(cmd, "reboot") || !strcmp(cmd, "reset")) {
    out.println("OK rebooting");
    g_reboot = true;
  } else {
    out.printf("ERR unknown command '%s' (type help)\n", cmd);
  }
}

}  // namespace commands
