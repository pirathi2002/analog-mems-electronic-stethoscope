"""Recording files.

Our recordings are WAV files at 4 kHz, int16:
    channel 0 = raw (DC removed, unfiltered), channel 1 = filtered
(BLE recordings only contain the filtered channel -> mono WAV).
A JSON side-car (same name, .json) stores the settings and the per-second
status log of the device (HR, levels, ...).

``load_reference`` reads BIOPAC (AcqKnowledge) exports for the comparison:
text (.txt/.csv), .wav, .mat and - if the ``bioread`` package is installed - .acq.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
from scipy.io import wavfile


def save_recording(path: Path, fs: int, filtered: np.ndarray, raw: np.ndarray | None = None,
                   meta: dict | None = None) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    to16 = lambda v: np.clip(np.round(np.nan_to_num(v) * 32767), -32767, 32767).astype(np.int16)  # noqa: E731
    if raw is not None and np.isfinite(raw).any():
        data = np.stack([to16(raw), to16(filtered)], axis=1)
        channels = ["raw", "filtered"]
    else:
        data = to16(filtered)
        channels = ["filtered"]
    wavfile.write(path, fs, data)
    m = dict(meta or {})
    m.update({"fs": fs, "channels": channels, "samples": int(len(filtered)),
              "full_scale": "int16 32767 = ADC full scale"})
    path.with_suffix(".json").write_text(json.dumps(m, indent=2, default=float))
    return path


def load_recording(path: Path):
    """Returns (fs, raw or None, filtered, meta). Values in ADC full-scale units."""
    path = Path(path)
    fs, data = wavfile.read(path)
    data = data.astype(np.float64) / 32767.0
    meta = {}
    side = path.with_suffix(".json")
    if side.exists():
        meta = json.loads(side.read_text())
    if data.ndim == 1:
        return fs, None, data, meta
    chans = meta.get("channels", ["raw", "filtered"])
    raw = data[:, chans.index("raw")] if "raw" in chans else None
    filt = data[:, chans.index("filtered")] if "filtered" in chans else data[:, -1]
    return fs, raw, filt, meta


def load_reference(path: Path, fs: float | None = None, column: int | None = None):
    """Load a reference (e.g. BIOPAC) recording. Returns (fs, signal)."""
    path = Path(path)
    ext = path.suffix.lower()
    if ext == ".wav":
        sr, d = wavfile.read(path)
        d = d.astype(np.float64)
        if d.ndim > 1:
            d = d[:, column or 0]
        return float(fs or sr), d
    if ext == ".mat":
        from scipy.io import loadmat

        m = loadmat(path)
        arrays = {k: np.asarray(v) for k, v in m.items() if not k.startswith("__")}
        if "isi" in arrays and fs is None:  # BIOPAC MATLAB export: isi = ms/sample
            fs = 1000.0 / float(arrays["isi"].squeeze())
        data = max((v for v in arrays.values() if v.dtype.kind in "fiu"), key=lambda v: v.size)
        data = np.atleast_2d(data)
        if data.shape[0] < data.shape[1]:
            data = data.T
        if fs is None:
            raise ValueError("sampling rate unknown - pass --ref-fs")
        return float(fs), data[:, column or 0].astype(np.float64)
    if ext == ".acq":
        import bioread  # optional dependency

        f = bioread.read_file(str(path))
        ch = f.channels[column or 0]
        return float(ch.samples_per_second), np.asarray(ch.data, dtype=np.float64)

    # Text export (AcqKnowledge "Save as text", CSV, ...)
    header_fs = None
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        m = re.search(r"([\d.]+)\s*msec/sample", line)
        if m:
            header_fs = 1000.0 / float(m.group(1))
        m = re.search(r"([\d.]+)\s*(?:Hz|samples/sec)", line, re.I)
        if m and header_fs is None:
            header_fs = float(m.group(1))
        parts = re.split(r"[,\t; ]+", line.strip())
        try:
            rows.append([float(p) for p in parts if p != ""])
        except ValueError:
            continue
    if not rows:
        raise ValueError(f"no numeric data in {path}")
    width = max(set(len(r) for r in rows), key=[len(r) for r in rows].count)
    data = np.array([r for r in rows if len(r) == width])
    fs = fs or header_fs
    if column is None:
        column = 0
        if width > 1:
            d0 = np.diff(data[:200, 0])
            if np.all(d0 > 0) and np.allclose(d0, d0[0], rtol=1e-3):  # first column is time
                if fs is None:
                    dt = d0[0]
                    fs = 1.0 / dt if dt < 0.05 else 1000.0 / dt  # seconds or ms
                column = 1
    if fs is None:
        raise ValueError("sampling rate unknown - pass --ref-fs")
    return float(fs), data[:, column]
