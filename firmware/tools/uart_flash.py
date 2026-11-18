#!/usr/bin/env python3
"""Flash the main board through the J1 UART header (no auto-reset lines).

Waits until you put the board in download mode (short P1, press SW1), reading the
ESP32 ROM boot message, then runs the arduino-cli upload immediately.

  python tools/uart_flash.py --port /dev/ttyUSB0 --fqbn <fqbn> --build-dir build/pcb-uart
(used by: make upload BOARD=pcb-uart)
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import serial

FW = Path(__file__).resolve().parents[1]


def wait_for_download(port: str, timeout: float) -> bool:
    s = serial.Serial(port, 115200, timeout=0.1)
    s.dtr = s.rts = False
    print(f"\n>>> Now: short P1, then press and release SW1 - or, with P1 shorted, unplug and replug\n"
          f"    the board's power (USB-C)   (waiting {timeout:.0f} s on {port}, Ctrl+C to cancel)\n", flush=True)
    buf, t0 = b"", time.time()
    while time.time() - t0 < timeout:
        try:
            chunk = s.read(4096)
        except serial.SerialException:
            # The adapter re-enumerated (USB glitch when the board is powered up). The boot
            # message is lost, but the chip may be in download mode: reopen and let esptool try.
            print("   (adapter reset on USB - reopening the port; will try to connect anyway)", flush=True)
            s.close()
            for _ in range(50):
                time.sleep(0.1)
                try:
                    s = serial.Serial(port, 115200, timeout=0.1)
                    break
                except serial.SerialException:
                    continue
            else:
                print("!!! adapter did not come back"); return False
            s.close()
            return True
        if chunk:
            buf += chunk
            for line in chunk.decode(errors="replace").splitlines():
                if line.strip():
                    print("   board:", line.strip(), flush=True)
        if b"waiting for download" in buf:
            s.close()
            return True
        if b"SPI_FAST_FLASH_BOOT" in buf:
            print("\n!!! The board started its program, so IO0 was not at GND.\n"
                  "    Keep P1 shorted while you press SW1 (check the jumper / P1 soldering).", flush=True)
            buf = b""
    s.close()
    print("\n!!! Nothing heard from the board. If SW1 was pressed, SW1 may not reset the chip:\n"
          "    power-cycle the board instead. Also check: board powered (USB-C), adapter RX -> J1-2,\n"
          "    adapter TX -> J1-3, GND -> J1-4, adapter jumper on 3.3 V.")
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--fqbn", required=True)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--timeout", type=float, default=180)
    a = ap.parse_args()
    if not wait_for_download(a.port, a.timeout):
        return 1
    print("\n>>> Download mode detected - flashing (you can remove P1 now)\n", flush=True)
    cli = shutil.which("arduino-cli") or str(Path.home() / ".local/bin/arduino-cli")
    r = subprocess.run([cli, "upload", "--fqbn", a.fqbn, "--input-dir", a.build_dir, "-p", a.port,
                        str(FW / "stethoscope")])
    if r.returncode == 0:
        print("\n>>> Done. Remove P1 and press SW1 to start the new firmware.")
    else:
        print("\n!!! Upload failed after download mode was reached: the PC -> board direction is the\n"
              "    problem: check adapter TX -> J1-3 (RX) and the trace to module pin 36 (RXD0).\n"
              "    Slower speed: make upload BOARD=pcb-uart UPLOAD_SPEED=115200")
    return r.returncode


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ncancelled")
        sys.exit(130)
