# Firmware and PC software

Firmware for the **ESP32-S3-WROOM-1-N16R8** main board (IC1), plus the PC software for real-time plotting, recording and evaluation.

**Owner:** Pirathi (branch `firmware/pirathi`)

| Part | Language | Runs on | Folder |
|---|---|---|---|
| Firmware: acquisition, digital filtering, HR, USB/BLE | C++ (Arduino core 3.x / ESP-IDF 5) | ESP32-S3 | [`stethoscope/`](stethoscope/) |
| Real-time viewer, recorder, analysis, filter design | Python 3 | PC | [`tools/`](tools/) |
| Unit tests of the firmware DSP | C++ + Python | PC | [`test/`](test/) |

## Hardware interface (main board V1 schematic)

| Signal | ESP32-S3 pin | Use in firmware |
|---|---|---|
| AFE output (U4 buffer, biased at Vref = 1.65 V) | **IO4** = ADC1_CH3 | DMA sampling at 16 kHz, 12 dB attenuation |
| USB D- / D+ (J2) | IO19 / IO20 | `Serial` = native USB-Serial/JTAG (flashing + data) |
| UART header J1 (1 3V3, 2 TX, 3 RX, 4 GND) | TXD0 / RXD0 | optional link (`LINK_USE_UART 1`, 921600 baud); adapter RX → J1-2, adapter TX → J1-3 |
| P1 boot jumper | IO0 | short to GND during reset = download mode (rarely needed) |
| SW1 | EN | reset |

The AFE gain is set with trimmer **R2** (U3 gain = 1 + R2 / 3 kΩ). The firmware reports clipping and the DC bias, so R2 can be adjusted while watching the viewer.

## Signal chain

```
ADC 16 kHz ─► 63-tap FIR (≤1 kHz pass, −62 dB ≥2 kHz) ─► ÷4 ─► DC blocker (0.5 Hz) ─► RAW 4 kHz
                                                                  │
                     ┌────────────────────────────────────────────┤
                     ▼                                            ▼
            50 Hz notch (Q 30)                       notch ─► BP 25–150 Hz (HR branch)
                     ▼                                            ▼
     Butterworth HP ─► Butterworth LP (mode)            Shannon-energy envelope (50 Hz)
                     ▼                                            ▼
                digital gain ─► FILTERED 4 kHz         8 s autocorrelation ─► HR (1 Hz)
```

| Mode | Band | Use |
|---|---|---|
| `heart` (default) | 25–400 Hz | standard PCG band: S1, S2, most murmurs |
| `bell` | 20–200 Hz | low-frequency sounds: S3, S4, low-pitched murmurs |
| `diaphragm` | 100–500 Hz | high-pitched sounds, lung sounds |
| `wide` | 20–1000 Hz | extended range |
| `raw` | none | unfiltered reference for raw-vs-filtered comparison |
| `custom` | `band <hp> <lp>` | any band |

The HP/LP sections are 4th-order Butterworth by default (`order 2|4|6`). They are designed on the MCU at runtime, so any band can be set without reflashing. **Heart-rate estimation:** the normalised Shannon energy envelope of the 25–150 Hz band is autocorrelated over the last 8 s every second. The strongest peak between 40 and 200 bpm gives the cardiac period, the peak height gives a confidence value, and the result is the median of the last 5 estimates. A window is rejected unless its envelope is **impulsive** (std/mean ≥ 0.8). Heart sounds measure 1.3–3 and steady noise below 0.5, so mains hum, whose close harmonics "beat" periodically, cannot produce a false heart rate.

Why 16 kHz → 4 kHz: oversampling lets the analog Sallen-Key filter (~1.6 kHz) plus the sharp digital FIR prevent aliasing. 4 kHz gives a comfortable margin above heart-sound content (< 600 Hz) and matches the 64 kbit/s budget in the design review.

## 1. Install the tools (once)

**Arduino toolchain.** Already installed on Pirathi's machine: `arduino-cli` in `~/.local/bin` + the *esp32 by Espressif* core 3.3.x.
```bash
arduino-cli config init --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index && arduino-cli core install esp32:esp32
```
Alternatively, Arduino IDE 2: *Boards Manager → "esp32 by Espressif Systems"*, then open `stethoscope/stethoscope.ino`.

**VS Code extensions:** *Arduino Community Edition* (`vscode-arduino.vscode-arduino-community`), *C/C++* (`ms-vscode.cpptools`) and *Serial Monitor* (`ms-vscode.vscode-serial-monitor`).

**Serial port permission (Linux):** `sudo usermod -aG dialout $USER`, then log out and back in.

**Python tools:**
```bash
cd firmware && make setup-tools        # creates tools/.venv and installs requirements.txt
sudo apt install libportaudio2         # only for "Listen" (audio playback) in the viewer
```

## 2. Build and flash

Board settings (already in the Makefile and in `stethoscope/sketch.yaml`):
*ESP32S3 Dev Module, USB Mode: Hardware CDC and JTAG, USB CDC On Boot: Enabled, Flash 16 MB, PSRAM: OPI PSRAM, Partition: 16M Flash (3MB APP/9.9MB FATFS)*.
If a board is fitted with a different S3 module, `make upload BOARD=auto` reads the real flash/PSRAM size from the chip.

```bash
cd firmware
make build                       # compile
make upload PORT=/dev/ttyACM0    # flash over the USB-C connector (J2)
make monitor                     # type: help
```
**Any ESP32-S3 board, detected automatically:**
```bash
make detect                      # shows port, chip, flash size, PSRAM
make upload BOARD=auto           # picks flash/PSRAM/partition and the PC link for that board
```
The port is found automatically (native USB first, then CH34x/CP210x/FTDI); use `PORT=/dev/ttyUSB0` to choose one when several boards are connected. A board on the ESP32 native USB port gets the USB link; a board on a USB-UART bridge gets the UART0 link at 921600. In VS Code, **Ctrl+Shift+B** runs the same upload (choose `auto`, `pcb` or `devkit`).

Other ESP32 families (classic ESP32, S2, C3, C6, H2) are detected and refused with a reason. The firmware relies on S3-specific hardware: two cores, ADC1 DMA on GPIO4 and BLE.

If the upload cannot connect, short the **P1** jumper (IO0 to GND), press **SW1** (EN), flash, then remove the jumper and press SW1 again.

### Programming through the J1 UART header (TX/RX)
Use a **3.3 V** USB-UART adapter (CP2102, CH340, FTDI; set its jumper to 3.3 V, not 5 V). J1 has no reset/boot lines, so download mode is entered by hand.

| Adapter | J1 pin |
|---|---|
| GND | 4 (GND) |
| RX  | 2 (TX, ESP32 output) |
| TX  | 3 (RX, ESP32 input) |
| 3V3 | 1, **only** if the board has no other power (USB-C/battery); adapter 3V3 pins are often too weak for BLE |

```bash
make upload BOARD=pcb-uart              # waits until you short P1 + reset (SW1, or unplug/replug power), then flashes
                                        # after flashing: remove P1, reset again
make monitor PORT=/dev/ttyUSB0          # 921600 baud, the PC link now runs over J1
```
Lower the speed if flashing fails over long wires: `make upload BOARD=pcb-uart UPLOAD_SPEED=115200`. With USB-C connected at the same time, give the port explicitly (`PORT=/dev/ttyUSB0`). The viewer and record tools also work over J1: `viewer.py --port /dev/ttyUSB0`.

#### Manual UART flashing (used on the main board)
If `make upload BOARD=pcb-uart` prints `adapter reset on USB` and then `No serial data received`, the USB-UART adapter re-enumerated when the board was reset. The script then lost the ROM boot message and guessed download mode. Flash by hand instead. This procedure flashed the main board through an FT232R adapter on `/dev/ttyUSB0`:

```bash
cd firmware
make build BOARD=pcb-uart                       # 1. compile (output in build/pcb-uart)

# 2. download mode: short P1, press + release SW1. Optional check of the ROM message:
tools/.venv/bin/python -m serial.tools.miniterm /dev/ttyUSB0 115200
#    expect "boot:0x0 (DOWNLOAD(USB/UART0))" and "waiting for download", then exit with Ctrl+]

# 3. flash (keep P1 shorted, do not press SW1)
ESPTOOL=~/.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool
B=build/pcb-uart
$ESPTOOL --chip esp32s3 --port /dev/ttyUSB0 --baud 115200 --before no-reset --after no-reset \
  write-flash -z --flash-mode keep --flash-freq keep --flash-size keep \
  0x0 $B/stethoscope.ino.bootloader.bin 0x8000 $B/stethoscope.ino.partitions.bin \
  0xe000 $B/boot_app0.bin 0x10000 $B/stethoscope.ino.bin
#    every part must end with "Hash of data verified."

# 4. run: remove P1, press SW1, then
make monitor PORT=/dev/ttyUSB0
```
With `--after no-reset` the chip stays in the bootloader after flashing. It is silent on UART and BLE until P1 is removed and SW1 is pressed. To check what is in flash without changing it: `$ESPTOOL --chip esp32s3 --port /dev/ttyUSB0 --before no-reset --after no-reset verify-flash 0x10000 $B/stethoscope.ino.bin` (in download mode).

If the adapter keeps dropping off USB when SW1 is pressed (`journalctl -k | grep "USB disconnect"`), check the power. Run the board from USB-C, a charger or the battery, and connect only TX, RX and GND to the adapter.

### Testing on an ESP32-S3 DevKit (before the PCB is assembled)
Tested on an **ESP32-S3-DevKitC N16R8** (16 MB flash, 8 MB *octal* PSRAM) plugged into its **COM** port (CH343 USB-UART):
```bash
make upload BOARD=devkit PORT=/dev/ttyACM0
make monitor                       # 921600 baud
```
`BOARD=devkit` selects octal PSRAM and 16 MB flash, and moves the PC link to UART0 at 921600 baud. The PC tools work unchanged. Without an AFE, IO4 floats: the status shows `bias ~60 mV` and many clipped samples, which is expected. To feed it a signal, connect IO4 to the middle of a 2 × 10 kΩ divider between 3V3 and GND (1.65 V bias) and AC-couple an audio source (phone headphone output through 1 µF + 10 kΩ) into that node. Keep the level small.

## 3. Use it

### Serial Monitor (any baud rate, line ending "Newline")
After boot the device prints one status line per second:
```
HR  72.4 bpm (conf 0.61) | level  -28.3 dBFS raw,  -30.1 dBFS filt | bias 1648 mV | clip 0 | fs 16000 Hz | heart
```

| Command | Effect |
|---|---|
| `help`, `status`, `info` | command list, JSON status, firmware/hardware info |
| `mode heart\|bell\|diaphragm\|wide\|raw` | listening filter |
| `band 30 250` | custom band (Hz) |
| `notch 50\|60\|off`, `gain 4`, `order 6` | mains notch, digital gain, filter order |
| `stream log\|plot\|bin\|off` | 1 line/s, Arduino Serial Plotter, binary (PC tools), quiet |
| `ble on\|off` | Bluetooth streaming (saved, applies after reboot) |
| `save`, `defaults`, `reboot` | store settings in flash, factory settings, restart |

`stream plot` sends `raw:…,filtered:…` at 500 Hz for the Arduino IDE Serial Plotter, as a quick check without Python.

### PC viewer (real-time plotting, recording, listening)
```bash
cd firmware/tools
.venv/bin/python viewer.py            # USB, port auto-detected
.venv/bin/python viewer.py --ble      # wireless (battery operation), raw + filtered
.venv/bin/python viewer.py --demo     # synthetic heart sounds, no hardware needed
```
The viewer shows raw and filtered PCG, the spectrum, HR with confidence, AFE bias, and clipping warnings. It has mode/notch/gain controls, a **Record** button (WAV saved to `tools/recordings/`) and **Listen** (headphones).

### Bluetooth LE
The device advertises as **`MEMS-Stetho`**. Service `6e5a0001-…`:

| Characteristic | Properties | Content |
|---|---|---|
| `…0005` AUDIO_RF | notify | uint32 sample index + int16 **raw[n] + filtered[n]** at 4 kHz (2 × 32 samples per 64-sample block, 128 kbit/s) |
| `…0002` AUDIO | notify | uint32 sample index + int16 filtered samples (64 kbit/s); only sent while AUDIO_RF is not subscribed |
| `…0003` STATUS | notify, read | JSON once per second, command replies |
| `…0004` CONTROL | write | the same text commands as on the serial port |

The viewer and `record.py --ble` subscribe to AUDIO_RF, so raw and filtered are both plotted and recorded over BLE. With firmware older than AUDIO_RF they fall back to the filtered channel. The firmware asks for a 7.5–15 ms connection interval and retries a notification for up to 20 ms when the controller buffers are full; packets still lost show up as `ble_drops` in the status and as zero-filled `lost_samples` in the recording's JSON.

Verified on hardware, with the Linux PC's built-in Bluetooth as the client:

| Board | Stream | Result |
|---|---|---|
| ESP32-S3 DevKit | filtered only | MTU 247, 30 s at 4 kHz, 0 lost samples, commands applied over BLE |
| Main board (UART build) | raw + filtered (AUDIO_RF) | MTU 247, 125 notifications/s × 32 samples = 4000 samples/s per channel; 40 s run: 155 680 of 155 680 samples received, 0 lost, `ble_drops` 0; repeated on a new connection (20 s, 0 lost) |

To repeat the check: `record.py -d 60 --ble` prints `lost N samples` at the end. The ESP32-S3 ADC runs ~0.16 % fast (measured 16026 Hz). Every status message reports the measured rate (`fs_adc`), and `analyze.py compare` uses it automatically.

## 4. Evaluation tasks (from the project brief)

| Deliverable | How |
|---|---|
| Digital filter design and simulation in Python | `make filters`: regenerates the FIR header and writes response plots + a pipeline demo to `tools/filter_design/` |
| HR from a 1-min recording | `record.py -d 60 -o rec/hr1` then `analyze.py report rec/hr1.wav --ref-bpm <pulse-ox value>` |
| HR accuracy over a 10-min window | `record.py -d 600 …` then `analyze.py report … --ref-bpm …` (MAE / RMSE of the 1 s HR track) |
| Raw vs. digitally filtered | `analyze.py report` (`waveforms_hr.png`, `spectrum.png`, SNR raw vs. filtered) |
| Characterisation with known-frequency signals | play tones from a speaker, record each, then `analyze.py tone t50.wav t100.wav t200.wav --freq 50 100 200` |
| Controlled noise / SNR | `analyze.py noise with_signal.wav noise_only.wav` |
| Comparison with the BIOPAC stethoscope | record the same source simultaneously; export BIOPAC as text/.mat; `analyze.py compare ours.wav biopac.txt` (lag, Pearson r of waveforms and envelopes, SNR and HR of both) |
| Repeatability | `analyze.py repeat r1.wav r2.wav r3.wav …` (mean, SD, CV) |
| Chest-piece / mic placement study | same protocol per variant, compare with `repeat` / `report` |

Recordings are 4 kHz int16 WAV files (ch0 raw, ch1 filtered) with a `.json` side-car that holds the settings and the device's per-second status log.

## 5. Tests
```bash
make test     # compiles stethoscope/src/dsp/*.cpp with g++ and checks it against tools/stetho/dsp.py
```
The tests check that the C++ output matches the SciPy reference, that the filter corners sit at −3 dB, that 50 Hz hum is attenuated by more than 30 dB, and that the HR estimate is within ±3 bpm at 50–150 bpm (including 6 dB SNR).

## Layout

```
firmware/
├── Makefile                  build / upload / monitor / test / viewer
├── stethoscope/              Arduino sketch (open this folder in Arduino IDE)
│   ├── stethoscope.ino       setup, DSP task, serial streaming
│   ├── sketch.yaml           arduino-cli board profile
│   └── src/
│       ├── config.h          pins, sample rates, BLE UUIDs, defaults
│       ├── dsp/              portable C++: biquads, FIR decimator, pipeline, HR estimator
│       ├── hal/              ESP32-S3 continuous ADC (DMA) + calibration
│       ├── link/             binary frame protocol, BLE GATT server
│       └── app/              commands, shared state, NVS settings
├── tools/                    PC software (Python)
│   ├── viewer.py  record.py  analyze.py  design_filters.py
│   └── stetho/               dsp reference, protocol decoder, transports, synthetic PCG
└── test/                     host tests for the DSP
```
