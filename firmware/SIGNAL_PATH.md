# Signal path: from the MEMS microphone to the PC

This document follows one heart sound through the whole system. It covers each filter in order: what it does, why it is there, its settings and its measured response. The firmware source for each stage is linked in its section.

All response values in the tables were computed from the PC reference DSP ([`tools/stetho/dsp.py`](tools/stetho/dsp.py)). That code uses the same formulas and coefficients as the firmware, and the host unit tests (`make test`) check that the C++ code matches it.

## Overview

```
 HEART SOUND (20-600 Hz)
        │
        ▼
 ┌──────────────────────── ANALOG (main board) ─────────────────────────┐
 │ MEMS microphone ─► U3 amplifier (gain = 1 + R2/3 kΩ) ─► Sallen-Key   │
 │                    biased at Vref = 1.65 V               LPF ~1.6 kHz│
 │                                                          ─► U4 buffer│
 └───────────────────────────────────────────────────────────┬──────────┘
                                                             ▼ IO4
 ┌──────────────────────── ESP32-S3 firmware ───────────────────────────┐
 │ [A] ADC 12 bit, 16 kHz (DMA)                                         │
 │        ▼                                                             │
 │ [B] FIR anti-alias low-pass (63 taps) + decimate ÷4 ─► 4 kHz         │
 │        ▼                                                             │
 │ [C] DC blocker (high-pass 0.5 Hz) ───────────────────► "RAW" output  │
 │        │                                                             │
 │        ├──────────── listening path ─────────┐                       │
 │        │                                     ▼                       │
 │        │                         [D] mains notch 50 Hz               │
 │        │                         [E] Butterworth high-pass (mode)    │
 │        │                         [F] Butterworth low-pass  (mode)    │
 │        │                         [G] digital gain ─► "FILTERED" output│
 │        │                                                             │
 │        └──────────── heart-rate path ───────┐                        │
 │                                             ▼                        │
 │                                 [H] mains notch 50 Hz                │
 │                                 [I] Butterworth band-pass 25-150 Hz  │
 │                                 [J] Shannon energy envelope (50 Hz)  │
 │                                 [K] autocorrelation (8 s) ─► HR      │
 │                                 [L] median of last 5 ─► "HR" output  │
 └──────────────────────────────────────┬───────────────────────────────┘
                                        ▼
            USB / UART (raw + filtered + status) or BLE (raw + filtered + status)
                                        ▼
                      PC viewer / recorder (tools/viewer.py, record.py)
```

The DSP code works on samples as floating-point numbers between −1.0 and +1.0, where ±1.0 is the full ADC range. Before sending, the raw and filtered outputs are converted to 16-bit integers (±32767 = ADC full scale).

| Signal | Sample rate | Meaning |
|---|---|---|
| ADC samples | 16 000 Hz | what the ADC measures on IO4 |
| **RAW** | 4 000 Hz | DC removed, otherwise unfiltered; the reference for comparisons |
| **FILTERED** | 4 000 Hz | after notch, band-pass and gain; what you listen to |
| **HR** | 1 per second | heart rate in bpm, with a confidence value |

---

## Analog front end (hardware, before the firmware)

| Part | Function |
|---|---|
| MEMS microphone | converts the chest-piece sound into a small AC voltage |
| U3 non-inverting amplifier | gain = 1 + R2 / 3 kΩ, adjusted with trimmer **R2**. The output is centred on **Vref = 1.65 V**, half of 3.3 V, so the waveform can swing both up and down inside the ADC's 0–3.3 V range |
| Sallen-Key low-pass, about 1.6 kHz | first anti-aliasing filter: it attenuates noise above about 1.6 kHz before sampling |
| U4 buffer | drives the ADC input (IO4) with low impedance |

The firmware checks the analog stage at every second. The status values `bias`, `clip` and `peak` are explained in [Diagnostics](#diagnostics-reported-every-second).

---

## [A] ADC sampling: 16 kHz, 12 bit
Source: [`hal/adc_sampler.cpp`](stethoscope/src/hal/adc_sampler.cpp)

| Setting | Value |
|---|---|
| Input | IO4 = ADC1 channel 3 (ADC1 keeps working while BLE is on) |
| Rate | 16 000 samples/s, continuous DMA (256-sample frames = 16 ms) |
| Resolution | 12 bit (codes 0 … 4095, mid-scale 2048) |
| Attenuation | 12 dB → input range about 0 … 3.1 V |

Each code is converted to the range −1.0 … +1.0 around mid-scale: `x = (code − 2048) / 2048`.

**Why 16 kHz rather than 4 kHz directly?** This is *oversampling*. Heart sounds stay below about 600 Hz, so 4 kHz would be enough in theory. But the analog low-pass at about 1.6 kHz is gentle, so noise between 2 kHz and 8 kHz still reaches the ADC. Sampling at 16 kHz keeps that noise out of the audio band, and the sharp digital FIR in [B] then removes it before the rate drops to 4 kHz.

---

## [B] FIR anti-aliasing low-pass + decimation ÷4
Source: [`dsp/fir_decimator.h`](stethoscope/src/dsp/fir_decimator.h), coefficients in [`dsp/fir_decim_coeffs.h`](stethoscope/src/dsp/fir_decim_coeffs.h), designed by [`tools/design_filters.py`](tools/design_filters.py)

**What it does:** a low-pass FIR filter, followed by keeping only 1 of every 4 samples (16 kHz → 4 kHz).

| Property | Value |
|---|---|
| Type | FIR (finite impulse response), window method, **Kaiser window β = 6** |
| Length | **63 taps** |
| Cutoff | 1500 Hz (−6 dB point) |
| Pass band | 0 – 1000 Hz, ripple **0.011 dB** (practically flat) |
| Stop band | ≥ 2000 Hz, attenuation **≥ 61.8 dB** |
| Delay | (63 − 1) / 2 = 31 samples at 16 kHz = **1.94 ms**, the same for all frequencies (linear phase) |

Equation (one output for every 4th input):

```
y[k] = Σ (i = 0 … 62) h[i] · x[4k + 3 − i]
```

**Why:** after decimation the new Nyquist frequency is 2000 Hz. Anything above 2000 Hz left in the signal would *alias*: it would fold back and appear as a false low-frequency sound. The FIR removes it first. An FIR filter is used here because its phase is linear, so every frequency is delayed by the same amount and the shape of S1/S2 is not distorted. The implementation only computes the outputs it keeps (one multiply-add per tap for every 4th sample), using a double-length ring buffer.

---

## [C] DC blocker: 1st-order high-pass at 0.5 Hz → RAW output
Source: [`dsp/pcg_pipeline.h`](stethoscope/src/dsp/pcg_pipeline.h) (`PcgPipeline::push`)

```
y[n] = x[n] − x[n−1] + R · y[n−1]        R = 1 − 2π · 0.5 / 4000 = 0.99921
```

| Frequency | 0.1 Hz | 0.5 Hz | 1 Hz | 5 Hz | ≥ 20 Hz |
|---|---|---|---|---|---|
| Gain | −14.2 dB | −3.0 dB | −1.0 dB | −0.04 dB | 0 dB |

**Why:** the ADC signal sits on the 1.65 V bias, a large constant (DC) value. The DC blocker removes it and also very slow drift, while it leaves the whole heart-sound band (≥ 20 Hz) untouched.

**The output of this stage is the RAW signal.** It is streamed and recorded unchanged, so it can be compared with the filtered signal and with the BIOPAC reference.

---

## Listening path: RAW → FILTERED

### [D] Mains notch filter, 50 Hz
Source: [`dsp/biquad.cpp`](stethoscope/src/dsp/biquad.cpp) (`designNotch`)

A second-order IIR (biquad) **notch**: it removes one exact frequency and passes the rest.

| Setting | Value |
|---|---|
| Centre | **50 Hz** (Sri Lanka mains). Command `notch 60` for 60 Hz countries, `notch off` to disable |
| Q | **30** → −3 dB width = 50 / 30 ≈ **1.7 Hz** (49.2 – 50.8 Hz) |

Design (RBJ Audio EQ Cookbook, bilinear transform), with w0 = 2π·f0/fs and α = sin(w0)/(2Q):

```
b = [1, −2·cos w0, 1] / (1 + α)        a = [1, −2·cos w0 / (1 + α), (1 − α) / (1 + α)]
```

| Frequency | 40 Hz | 50 Hz | 51 Hz | 60 Hz | 100 Hz | 150 Hz |
|---|---|---|---|---|---|---|
| Gain (heart mode, whole chain) | −0.1 dB | **< −290 dB** (removed) | −2.3 dB | 0 dB | 0 dB | 0 dB |

**Why:** power-line hum couples into the microphone wiring and the amplifier. The high Q makes the notch so narrow that the heart sound around it is almost not affected.

> **Limitation:** only the 50 Hz fundamental is removed. The harmonics at 100, 150, 200, 250, 300 and 350 Hz pass through (0 dB in the table above). They are usually small on battery power but can be visible when a cable connects the board to a mains-powered laptop. Check the spectrum plot in the viewer. If spikes appear at multiples of 50 Hz, harmonic notches can be added (one biquad per harmonic).

### [E] + [F] Butterworth high-pass and low-pass (the filter mode)
Source: [`dsp/biquad.cpp`](stethoscope/src/dsp/biquad.cpp), configured in [`dsp/pcg_pipeline.cpp`](stethoscope/src/dsp/pcg_pipeline.cpp) (`configure`)

Together, the high-pass [E] and the low-pass [F] form a **band-pass** that keeps only the frequencies of interest.

| Setting | Value |
|---|---|
| Type | **Butterworth** (maximally flat pass band, no ripple) |
| Order | **4** by default (command `order 2`, `4` or `6`) → slope **24 dB/octave** for each corner |
| Built as | N/2 cascaded biquads with the Butterworth pole Q values (order 4: Q = 0.541 and 1.307) |
| Designed | on the ESP32 at runtime, so any band can be set without reflashing |

The corner frequencies depend on the **filter mode** (command `mode …`):

| Mode | High-pass | Low-pass | Used for |
|---|---|---|---|
| `heart` (default) | 25 Hz | 400 Hz | standard PCG band: S1, S2, most murmurs |
| `bell` | 20 Hz | 200 Hz | low-frequency sounds: S3, S4, low-pitched murmurs |
| `diaphragm` | 100 Hz | 500 Hz | high-pitched sounds, lung sounds |
| `wide` | 20 Hz | 1000 Hz | extended range |
| `custom` | your value | your value | command `band <hp> <lp>`, e.g. `band 30 250` |
| `raw` | off | off | notch, high-pass and low-pass all skipped: FILTERED = RAW × gain |

Response of the complete listening chain [D]+[E]+[F] at order 4 (dB):

| Mode | 10 Hz | 20 Hz | 25 Hz | 50 Hz | 100 Hz | 200 Hz | 300 Hz | 400 Hz | 500 Hz | 800 Hz | 1000 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heart | −31.8 | −8.4 | **−3.0** | notch | 0.0 | 0.0 | −0.4 | **−3.0** | −9.0 | −28.0 | −39.1 |
| bell | −24.1 | **−3.0** | −0.7 | notch | 0.0 | **−3.0** | −14.6 | −25.0 | −33.4 | −52.9 | −64.0 |
| diaphragm | −80.1 | −56.0 | −48.2 | notch | **−3.0** | 0.0 | −0.1 | −0.6 | **−3.0** | −19.6 | −30.6 |
| wide | −24.1 | **−3.0** | −0.7 | notch | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | −0.3 | **−3.0** |

The corner frequencies (bold) are at −3 dB, as expected for Butterworth filters.

**Why:**
- **High-pass:** removes low-frequency noise such as chest-piece movement, rubbing, breathing and the residual DC.
- **Low-pass:** removes hiss and high-frequency noise above the heart-sound band.
- **Butterworth:** it has a flat pass band, so heart sounds inside the band are not coloured. An IIR filter needs far fewer operations than an FIR with the same sharpness, which matters at 4 kHz on a microcontroller.

### [G] Digital gain → FILTERED output
`filtered = band-pass output × gain` (command `gain 4`, default 1). This only changes the loudness of the FILTERED signal. If the analog gain is too high, the ADC clips, and digital gain cannot repair that, so set the level with trimmer R2 first.

---

## Heart-rate path: RAW → HR

This path runs in parallel with the listening path and **always uses its own fixed band**. The heart rate therefore keeps working whatever listening mode is selected.

### [H] Mains notch, 50 Hz
The same notch as [D], and it follows the same `notch` setting.

### [I] Butterworth band-pass, 25–150 Hz (4th order HP + 4th order LP)

| Frequency | 10 Hz | 25 Hz | 50 Hz | 100 Hz | 150 Hz | 200 Hz | 300 Hz | 500 Hz |
|---|---|---|---|---|---|---|---|---|
| Gain (with notch) | −31.8 | **−3.0** | notch | −0.2 | **−3.0** | −10.5 | −24.6 | −43.5 |

**Why this band:** most of the energy of S1 and S2 lies between 25 and 150 Hz. Murmurs and noise above 150 Hz would confuse the beat detection, so they are removed here, even though they are kept for listening.

### [J] Shannon energy envelope (50 Hz)
Source: [`dsp/hr_estimator.cpp`](stethoscope/src/dsp/hr_estimator.cpp)

1. Normalise: `x = |y| / max|y|`, using the running maximum over the last 3 s.
2. Shannon energy: `E = −x² · ln(x²)`.
3. Average E over 20 ms (80 samples) → one envelope value every 20 ms (50 Hz).

**Why Shannon energy:** it emphasises medium-intensity sounds (S1, S2). It suppresses both low-level background noise (small x → E ≈ 0) and single loud spikes (x ≈ 1 → E ≈ 0). The result is a smooth curve with one bump per heart sound.

### [K] Autocorrelation over 8 s
Every second, the firmware computes the autocorrelation of the last 8 s of envelope (400 values). A heartbeat repeats, so the autocorrelation has a peak at a lag equal to one cardiac period.

- The search range is lag 15 … 75 envelope samples = 0.3 … 1.5 s = **200 … 40 bpm**.
- Parabolic interpolation around the highest peak gives sub-sample accuracy.
- `HR = 60 / period`, and **confidence** = peak height R(T)/R(0), between 0 and 1.

A window is **rejected** when any of these is true:
- **pulsatility** (envelope std/mean) < 0.8. Heart sounds give 1.3–3, steady noise gives < 0.5. This also stops mains hum, whose close harmonics "beat" periodically, from producing a false heart rate.
- confidence < 0.15
- the result is outside 40–200 bpm

### [L] Median of the last 5 estimates → HR output
The reported HR is the **median** of the last 5 valid estimates. A median ignores single wrong values (for example a cough), whereas an average would be pulled towards them.

---

## Diagnostics reported every second

The status line, the JSON status and the viewer panel all report these values:

| Value | How it is computed | Good value | If not |
|---|---|---|---|
| `bias` (AFE bias) | mean ADC code over 1 s, converted to mV with the factory calibration | **about 1650 mV** | check Vref, U3/U4 supply, the IO4 connection |
| `clip` | samples per second within 8 codes of 0 or 4095 | **0** | fix the bias first, then turn R2 down |
| `level raw` / `level filtered` | RMS of RAW / FILTERED in dBFS (0 dBFS = full scale) | heart sounds about −40 to −20 dBFS | too low: raise R2; near 0: lower R2 |
| `peak` | largest \|RAW\| as a percentage of full scale | < 50 % | lower R2 |
| `fs` / ADC rate | ADC samples counted per second | about 16 000 Hz (this ADC measures about 16 026) | — |
| `lost samples` | gaps in the sample index received by the PC | 0 | move closer (BLE) or check the cable |

---

## PC noise reduction (viewer)
Source: [`tools/stetho/denoise.py`](tools/stetho/denoise.py). It runs on the PC on the **RAW** stream, so it needs the firmware with the raw + filtered BLE stream (or the USB/UART link).

```
RAW ─► [M] pulse / burst remover ─► [N] spectral subtraction ─► [D][E][F][G] same listening chain as the device ─► DENOISED
                    │                          │
                    │                          └──────────────► [O] HR (PC): band 25-150 Hz ─► envelope ─► autocorrelation
                    └─ burst times ─► [P] 1-s status lock ──────────────────► (status-burst windows left out of [O])
```

### What the interference looks like
Measured in the first recordings from the main board, with the chest-piece on the body and Bluetooth streaming:

| Source | Shape | Timing | Share of the noise |
|---|---|---|---|
| Bluetooth connection events | positive square pulse, **2.5 ms**, two humps | every 10–12.5 ms (about 55/s), jittered | the main part: lines at 88.75 Hz and all its multiples in the spectrum |
| Status message (1 per second) | longer burst, **about 8 ms** (several packets), then a 20–40 ms baseline recovery | exactly 1.000 s (`periodicStatus()`, `millis()`), jitter ±4 ms | appears as a false **60 bpm** in the heart rate |

The pulses grow **with the analog gain** (13× when R2 was turned up 13×). The interference therefore enters **before** the amplifier U3: through the MEMS microphone supply, Vref, or the microphone wiring. This also explains why raising R2 does not improve the signal-to-noise ratio. The lasting fix is in hardware: separate filtering (RC/LC or LDO) of the analog supply and Vref, and more bulk capacitance at the ESP32. The PC stage works around it.

### [M] Pulse and burst remover
- **Calibration** (chest-piece off the body): finds every steep rising edge (`x[n] − x[n−2]` above 8 × the robust noise level) and takes the **median** of all pulses as the **template** (7 ms).
- **Live:** a rising edge above the threshold (0.35 × typical pulse edge) marks a pulse. Heart sounds are far too slow to produce such an edge: the steepest heart-sound slope is about 1/30 of a pulse edge.
  - **Normal pulse** (rise to fall ≤ 4 ms, template fits): subtract `amplitude × template`, fitted by least squares at the best of 3 sub-sample positions. The heart sound underneath is kept.
  - **Long burst** (> 4 ms, e.g. the status message): cut from the rising to the last falling edge (≤ 20 ms) and bridged with a straight line between the baselines before and after.
- The threshold follows the height of the recent pulses. Turning R2 up or down therefore needs no new calibration; the measured ratio is shown as "analog gain ×…".
- Latency 112 samples, 28 ms.

### [N] Spectral subtraction (background noise)
- Calibration: mean noise power spectrum of the calibration recording after [M] (512-sample frames, 128 ms, 7.8 Hz bins).
- Live: each frame (sqrt-Hann, 50 % overlap) is scaled per frequency bin by `G = sqrt(max(1 − α·N/|X|², floor²))`. α is the **Strength** slider (default 1.5) and floor = −20 dB, which limits the "musical noise" of spectral subtraction. The noise spectrum is scaled by the gain ratio from [M].
- With α = 0 the stage passes the signal through unchanged (perfect reconstruction). The tests check that, with [M] and [N] switched off, the denoised output equals the device's listening chain exactly.
- Latency 256 samples, 64 ms.

### [O] Heart rate on the PC
Same algorithm as the firmware ([I]–[L]): 25–150 Hz band, Shannon envelope, 8 s autocorrelation, median of the last 5. Two additions:
- **Max expected HR** (default 120 bpm). When S1 and S2 are similar, the S1→S2 gap (about 0.4 s at rest) can look like a full beat and double the rate. The limit stops that at rest; raise it after exercise.
- **Status-burst windows left out** ([P]): the envelope in a 15 ms-before / 80 ms-after window around each predicted status burst is replaced by the typical envelope level, so the 1-per-second burst cannot become a 60 bpm "heartbeat".

### [P] 1-second status lock
Long bursts from [M] are folded modulo the status period (1 s = `fs_adc / 4` samples, about 4006.4). The phase where most of them fall, within ±10 ms, is the status phase. With it, every status burst can be predicted, including those [M] did not detect. Typical: 65–88 % of the bursts detected, lock in more than 95 % of the estimates.

### Results on the first recordings
Calibration was taken from the first 5 s of each recording, because there was no separate noise recording yet:

| Recording | Noise removed | HR windows accepted (before → after) |
|---|---|---|
| heart (athlete, 158 s) | 11–23 dB | 0 % → about 60 %, median about 70 bpm |
| hand taps (73 s, 136 s) | 6–15 dB | 0 % → 56–100 % (the tap rhythm, about 60/min) |
| phone playing a heart sound | 1–4 dB | 0 % → about 50 % (phone speakers barely reproduce < 150 Hz) |

The true heart rate during the heart recording was not measured. Count the pulse or use a pulse oximeter during the next recording to check the PC value.

## How to see each stage for yourself

| What | How |
|---|---|
| Frequency response plots of every filter | `make filters` → PNG files in `tools/filter_design/` |
| RAW vs FILTERED live, with spectrum | `make viewer` (USB/UART) or `viewer.py --ble` |
| Switch modes live and hear the difference | type `mode bell`, `mode raw`, … in the viewer's command box, tick **Listen** |
| Offline analysis of a recording | `python analyze.py report <file.wav>` |
| Check that firmware and reference DSP match | `make test` |
| Try everything without hardware | `make demo` (synthetic heart sounds with 50 Hz hum) |
