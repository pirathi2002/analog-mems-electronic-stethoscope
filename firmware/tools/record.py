#!/usr/bin/env python3
"""Record heart sounds from the stethoscope to a WAV file (headless).

Examples:
    python record.py -d 60                      # 1-minute recording (HR task)
    python record.py -d 600 -o rec/subject1     # 10-minute window
    python record.py -d 30 --mode bell          # set filter mode first
    python record.py -d 20 --ble                # over Bluetooth (raw + filtered)
    python record.py -d 10 --demo               # synthetic data, no hardware

Output: <name>.wav (4 kHz int16, ch0 raw, ch1 filtered) + <name>.json
(settings + the device's 1 Hz status log, including its HR estimates).
"""
from __future__ import annotations

import argparse
import queue
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from stetho import dsp
from stetho.recio import save_recording
from stetho.transport import StreamAssembler, make_transport


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--duration", type=float, default=60.0, help="seconds (default 60)")
    ap.add_argument("-o", "--out", type=Path, help="output path without extension")
    ap.add_argument("--port", help="serial port (auto-detected by default)")
    ap.add_argument("--ble", action="store_true", help="use Bluetooth LE instead of USB")
    ap.add_argument("--ble-address", help="BLE address (default: scan for MEMS-Stetho)")
    ap.add_argument("--demo", action="store_true", help="synthetic signal, no hardware")
    ap.add_argument("--mode", choices=dsp.MODE_NAMES[:-1], help="set filter mode before recording")
    ap.add_argument("--notch", help="off | 50 | 60")
    ap.add_argument("--gain", type=float)
    ap.add_argument("--label", default="", help="free text stored in the JSON (site, subject, ...)")
    args = ap.parse_args()

    kind = "demo" if args.demo else "ble" if args.ble else "serial"
    tr = make_transport(kind, args.port, args.ble_address)
    tr.start()
    for cmd, val in (("mode", args.mode), ("notch", args.notch), ("gain", args.gain)):
        if val is not None:
            tr.send(f"{cmd} {val}")

    out = args.out or Path("recordings") / datetime.now().strftime("pcg_%Y%m%d_%H%M%S")
    asm = StreamAssembler()
    raws, filts, statuses = [], [], []
    target = int(args.duration * dsp.FS)
    got = 0
    t0 = time.monotonic()
    started = False
    print(f"recording {args.duration:.0f} s via {kind} -> {out}.wav  (Ctrl+C to stop early)")
    try:
        while got < target:
            try:
                m = tr.messages.get(timeout=5.0)
            except queue.Empty:
                print("\nno data for 5 s - check the connection", file=sys.stderr)
                break
            if m.kind == "audio":
                if not started:  # discard ~0.5 s of filter settling after (re)configuration
                    if time.monotonic() - t0 < 0.5:
                        continue
                    started = True
                r, f = asm.add(m.audio)
                raws.append(r)
                filts.append(f)
                got += len(f)
            elif m.kind == "status":
                m.status["rec_t"] = got / dsp.FS
                statuses.append(m.status)
                hr = f"{m.status.get('hr', 0):5.1f} bpm" if m.status.get("hr_ok") else "  --- bpm"
                print(f"\r{got / dsp.FS:6.1f}/{args.duration:.0f} s   HR {hr}   "
                      f"level {m.status.get('rms', -99):6.1f} dBFS   clip {m.status.get('clip', 0)}   ",
                      end="", flush=True)
            elif m.kind == "text":
                print(f"\n[device] {m.text}")
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        tr.stop()
    print()
    if not filts:
        print("nothing recorded", file=sys.stderr)
        return 1
    filt = np.concatenate(filts)[:target]
    raw = np.concatenate(raws)[:target] if tr.has_raw else None
    meta = {"created": datetime.now().isoformat(timespec="seconds"), "source": kind,
            "label": args.label, "lost_samples": asm.lost,
            "settings": statuses[-1] if statuses else {}, "status_log": statuses}
    p = save_recording(out.with_suffix(".wav"), dsp.FS, filt, raw, meta)
    print(f"saved {p} ({len(filt) / dsp.FS:.1f} s, lost {asm.lost} samples)")
    print(f"analyse with:  python analyze.py report {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
