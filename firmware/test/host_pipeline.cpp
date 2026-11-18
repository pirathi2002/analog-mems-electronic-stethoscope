// Host (PC) harness that runs the *firmware* DSP code on a file of ADC codes.
//
//   host_pipeline <codes.u16> <mode> <notch_hz> <gain> <out.f32>
//
// codes.u16 : little-endian uint16 ADC codes sampled at 16 kHz
// out.f32   : float32 pairs [raw, filtered] at 4 kHz
// stdout    : one CSV line per HR update: sample_index,bpm,bpm_instant,confidence,valid,pulsatility
//
// Built and driven by firmware/test/test_dsp.py.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../stethoscope/src/config.h"
#include "../stethoscope/src/dsp/fir_decim_coeffs.h"
#include "../stethoscope/src/dsp/pcg_pipeline.h"

int main(int argc, char** argv) {
  if (argc != 6) {
    std::fprintf(stderr, "usage: %s codes.u16 mode notch_hz gain out.f32\n", argv[0]);
    return 2;
  }
  std::FILE* in = std::fopen(argv[1], "rb");
  if (!in) return 3;
  std::vector<uint16_t> codes;
  uint16_t buf[4096];
  size_t n;
  while ((n = std::fread(buf, sizeof(uint16_t), 4096, in)) > 0) codes.insert(codes.end(), buf, buf + n);
  std::fclose(in);

  dsp::PipelineSettings s;
  if (!dsp::parseMode(argv[2], &s.mode)) {
    std::fprintf(stderr, "bad mode %s\n", argv[2]);
    return 4;
  }
  s.notch_hz = static_cast<float>(std::atof(argv[3]));
  s.gain = static_cast<float>(std::atof(argv[4]));

  static dsp::PcgPipeline pipe;  // large object: keep off the stack
  pipe.begin(ADC_SAMPLE_RATE_HZ, kFirDecimTaps, FIR_DECIM_NUM_TAPS, DECIMATION);
  pipe.configure(s);
  pipe.reset();

  std::FILE* out = std::fopen(argv[5], "wb");
  if (!out) return 5;
  uint32_t updates = 0;
  uint32_t idx = 0;
  for (uint16_t c : codes) {
    float raw, filt;
    if (pipe.push(adc_code_to_float(c), &raw, &filt)) {
      float pair[2] = {raw, filt};
      std::fwrite(pair, sizeof(float), 2, out);
      ++idx;
      const dsp::HrResult& hr = pipe.hr();
      if (hr.updates != updates) {
        updates = hr.updates;
        std::printf("%u,%.3f,%.3f,%.4f,%d,%.4f\n", idx, hr.bpm, hr.bpm_instant, hr.confidence,
                    hr.valid ? 1 : 0, hr.pulsatility);
      }
    }
  }
  std::fclose(out);
  return 0;
}
