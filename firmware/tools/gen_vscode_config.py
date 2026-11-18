#!/usr/bin/env python3
"""Generate .vscode/c_cpp_properties.json for the ESP32-S3 firmware.

VS Code's C/C++ IntelliSense cannot read the ESP32 Arduino core's response
files (@.../flags/includes with -iprefix), so it cannot find Arduino.h,
FreeRTOS, ESP-IDF headers, ... and shows red "include errors".

This script takes the exact compiler, include paths and defines from the
compile database written by `make build` (firmware/build/<board>/compile_commands.json),
expands the response files, and writes a configuration for each board.

Usage (from firmware/):  make vscode      (runs `make build` first if needed)
"""
from __future__ import annotations

import json
import os
import shlex
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
REPO = FW.parent
HOME = str(Path.home())


def expand(args: list[str]) -> list[str]:
    """Expand @response-file arguments recursively."""
    out: list[str] = []
    for a in args:
        if a.startswith("@") and Path(a[1:]).is_file():
            out += expand(shlex.split(Path(a[1:]).read_text()))
        else:
            out.append(a)
    return out


def config_for(board: str) -> dict | None:
    db_path = FW / "build" / board / "compile_commands.json"
    if not db_path.exists():
        return None
    db = json.loads(db_path.read_text())
    entry = next(e for e in db if e["file"].endswith("stethoscope.ino.cpp"))
    args = expand(entry["arguments"])
    compiler = args[0]
    includes: list[str] = []
    defines: list[str] = []
    prefix = ""
    i = 1
    while i < len(args):
        a = args[i]
        nxt = args[i + 1] if i + 1 < len(args) else ""
        if a == "-iprefix":
            prefix, i = nxt, i + 2
            continue
        if a.startswith("-iwithprefixbefore") or a.startswith("-iwithprefix"):
            rel = a.split("prefixbefore", 1)[-1] if "before" in a else a.split("prefix", 1)[-1]
            if not rel:
                rel, i = nxt, i + 1
            includes.append(os.path.normpath(prefix + rel))
        elif a in ("-I", "-isystem"):
            includes.append(nxt)
            i += 1
        elif a.startswith("-I"):
            includes.append(a[2:])
        elif a.startswith("-isystem"):
            includes.append(a[len("-isystem"):])
        elif a == "-D":
            defines.append(nxt)
            i += 1
        elif a.startswith("-D"):
            defines.append(a[2:])
        i += 1
    # The sketch is compiled from a copy in the build folder; point at the real sources instead.
    includes = [p for p in includes if "/build/" not in p]
    includes.insert(0, "${workspaceFolder}/firmware/stethoscope")
    seen: set[str] = set()
    includes = [p for p in includes if not (p in seen or seen.add(p))]

    def portable(s: str) -> str:
        return s.replace(HOME, "${env:HOME}")

    return {
        "name": f"ESP32-S3 ({board})",
        "compilerPath": portable(compiler),
        "compilerArgs": ["-mlongcalls"],
        "intelliSenseMode": "gcc-x86",  # xtensa is not a known mode; gcc semantics are what matter
        "cStandard": "gnu17",
        "cppStandard": "gnu++2b",
        "includePath": [portable(p) for p in includes],
        "defines": [portable(d) for d in defines],
        "browse": {"limitSymbolsToIncludedHeaders": True},
    }


def main() -> int:
    configs = [c for c in (config_for(b) for b in ("pcb", "devkit")) if c]
    if not configs:
        print("no build found - run `make build` (and/or `make build BOARD=devkit`) first", file=sys.stderr)
        return 1
    # A host config so firmware/test/*.cpp (built with the PC's g++) is also error-free.
    configs.append({
        "name": "Host tests (PC g++)",
        "compilerPath": "/usr/bin/g++",
        "intelliSenseMode": "linux-gcc-x64",
        "cppStandard": "c++17",
        "includePath": ["${workspaceFolder}/firmware/stethoscope/src"],
    })
    out = REPO / ".vscode" / "c_cpp_properties.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps({"version": 4, "configurations": configs}, indent=2) + "\n")
    print(f"wrote {out.relative_to(REPO)}: " + ", ".join(c["name"] for c in configs))
    for c in configs[:-1]:
        print(f"  {c['name']}: {len(c['includePath'])} include paths, {len(c['defines'])} defines")
    return 0


if __name__ == "__main__":
    sys.exit(main())
