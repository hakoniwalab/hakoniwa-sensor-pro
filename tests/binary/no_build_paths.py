#!/usr/bin/env python3
"""The binary set carries no path of the machine it was built on.

1. No file of the binary set contains a home-directory or Homebrew path (/Users/, /home/, /opt/homebrew).
2. The shared library (the external sensor plugin) has no run-time search path (macOS LC_RPATH,
   Linux RPATH/RUNPATH): the host supplies the Hakoniwa, MuJoCo and GLFW libraries.

    python3 no_build_paths.py <binary set>
"""
import platform
import re
import subprocess
import sys
from pathlib import Path

BUILD_PATH = re.compile(rb"/Users/|/home/|/opt/homebrew")


def main():
    dist = Path(sys.argv[1])
    failures = 0
    for f in sorted(p for p in dist.rglob("*") if p.is_file()):
        hits = sorted({m.group(0).decode() for m in BUILD_PATH.finditer(f.read_bytes())})
        if hits:
            print(f"FAIL: {f.relative_to(dist)} contains {hits}")
            failures += 1
    plugin = dist / "lib" / "hakoniwa_mujoco_sensors_plugin.so"
    if platform.system() == "Darwin":
        out = subprocess.run(["otool", "-l", str(plugin)], check=True, capture_output=True, text=True).stdout
        rpath = "LC_RPATH" in out
    else:
        out = subprocess.run(["readelf", "-d", str(plugin)], check=True, capture_output=True, text=True).stdout
        rpath = "RPATH" in out or "RUNPATH" in out
    print(f"{plugin.name}: run-time search path {'present' if rpath else 'none'}")
    if rpath:
        failures += 1
    print("PASS" if failures == 0 else f"FAIL: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
