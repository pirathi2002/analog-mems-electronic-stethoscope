#!/usr/bin/env python3
"""Find the connected ESP32 board and choose the right build settings for it.

Used by `make upload BOARD=auto` (and `make detect`). It
  1. finds the serial port (native ESP32-S3 USB, CH34x, CP210x, FTDI ...),
  2. asks the chip itself with esptool: chip type, flash size, PSRAM size,
  3. prints the arduino-cli FQBN and the PC-link option for that board:
       native USB port (VID 303A)  -> PC link = USB-Serial/JTAG  (like our PCB)
       USB-UART bridge (CH343 ...) -> PC link = UART0 921600     (like a DevKitC "COM" port)

The firmware needs an ESP32-S3 (dual core, ADC1 DMA on GPIO4, BLE). Other chips
are recognised and rejected with an explanation instead of failing mid-upload.

  python tools/detect_board.py            human-readable report
  python tools/detect_board.py --make     one line for the Makefile: PORT FQBN EXTRA_FLAGS NAME
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys

from serial.tools import list_ports

USB_BRIDGES = {0x303A: "ESP32 native USB", 0x1A86: "CH34x USB-UART", 0x10C4: "CP210x USB-UART",
               0x0403: "FTDI USB-UART"}

WHY_NOT = {
    "ESP32-S2": "has no Bluetooth and a single core",
    "ESP32-C3": "is single core (the DSP and BLE tasks are pinned to two cores) and maps GPIO4 to a different ADC channel",
    "ESP32-C6": "is single core and maps GPIO4 to a different ADC channel",
    "ESP32-H2": "has no Wi-Fi, a single core and a different ADC",
    "ESP32-C2": "is single core with a different ADC",
    "ESP32-P4": "has no radio",
    "ESP32": "(classic) uses a different ADC DMA format and pin map; GPIO4 is on ADC2, which cannot run with BLE",
}


def find_ports() -> list[tuple[str, str]]:
    found = []
    for p in list_ports.comports():
        if p.vid in USB_BRIDGES:
            found.append((p.device, USB_BRIDGES[p.vid]))
    found.sort(key=lambda x: x[1] != "ESP32 native USB")  # prefer native USB
    return found


def probe(port: str) -> str:
    """Run esptool flash-id (connects, prints chip/features/flash, then resets the chip)."""
    r = subprocess.run([sys.executable, "-m", "esptool", "--port", port, "flash-id"],
                       capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        if "Permission denied" in r.stdout + r.stderr:
            raise SystemExit(f"No permission to open {port}. You are in the 'dialout' group but this login session "
                             "started before that: log out and back in (or restart the PC), then retry.\n"
                             "(Quick test without logging out:  sg dialout -c 'make upload BOARD=auto')")
        tail = (r.stdout + r.stderr).strip().splitlines()[-3:]
        raise SystemExit(f"esptool could not talk to {port}:\n  " + "\n  ".join(tail) +
                         "\nTry: hold BOOT (or short P1 on our PCB), press RESET, release BOOT, then retry.")
    return r.stdout


def parse(text: str) -> dict:
    chip = re.search(r"Chip type:\s+(ESP32[-\w]*)", text) or re.search(r"Chip is (ESP32[-\w]*)", text)
    flash = re.search(r"Detected flash size:\s+(\d+)MB", text)
    psram = re.search(r"Embedded PSRAM (\d+)MB", text)
    return {"chip": chip.group(1) if chip else "unknown",
            "flash_mb": int(flash.group(1)) if flash else 0,
            "psram_mb": int(psram.group(1)) if psram else 0}


def settings(info: dict, native_usb: bool) -> tuple[str, str, str]:
    if info["chip"] != "ESP32-S3":
        reason = WHY_NOT.get(info["chip"], "is not supported")
        raise SystemExit(f"Detected {info['chip']}: this firmware needs an ESP32-S3; {info['chip']} {reason}.")
    f = info["flash_mb"]
    if f >= 16:
        flash, part = "16M", "app3M_fat9M_16MB"
    elif f == 8:
        flash, part = "8M", "default_8MB"
    elif f == 4:
        flash, part = "4M", "default"
    else:
        raise SystemExit(f"Flash size {f} MB not supported (need >= 4 MB).")
    # S3R2 = quad PSRAM, S3R8/R16 = octal PSRAM
    psram = "disabled" if info["psram_mb"] == 0 else ("enabled" if info["psram_mb"] <= 2 else "opi")
    cdc = "cdc" if native_usb else "default"
    fqbn = (f"esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot={cdc},FlashSize={flash},"
            f"PSRAM={psram},PartitionScheme={part},CPUFreq=240")
    extra = f"-DLINK_USE_UART={0 if native_usb else 1}"
    name = f"s3-{flash}-psram{info['psram_mb']}-{'usb' if native_usb else 'uart'}"
    return fqbn, extra, name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto)")
    ap.add_argument("--make", action="store_true", help="print one line for the Makefile")
    a = ap.parse_args()

    if a.port and a.port != "auto":
        port, kind = a.port, next((k for d, k in find_ports() if d == a.port), "serial port")
    else:
        ports = find_ports()
        if not ports:
            raise SystemExit("No ESP32 found: plug the board in (use a data USB cable) and check `ls /dev/ttyACM* /dev/ttyUSB*`.")
        port, kind = ports[0]
        if len(ports) > 1 and not a.make:
            print(f"several boards connected, using {port}; choose another with PORT=...", file=sys.stderr)
    native = kind == "ESP32 native USB"
    info = parse(probe(port))
    fqbn, extra, name = settings(info, native)
    if a.make:
        print(port, fqbn, extra, name)
    else:
        print(f"port      : {port} ({kind})")
        print(f"chip      : {info['chip']}, flash {info['flash_mb']} MB, PSRAM {info['psram_mb']} MB")
        print(f"PC link   : {'native USB (Serial)' if native else 'UART0 921600 baud'}")
        print(f"FQBN      : {fqbn}")
        print(f"upload    : make upload BOARD=auto")
    return 0


if __name__ == "__main__":
    sys.exit(main())
