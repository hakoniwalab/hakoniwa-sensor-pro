#!/usr/bin/env python3
"""Build the binary set from the private sources at the revisions in config/sources.json.

The checkouts must be at those revisions with no changes to tracked files. The result is
dist/<platform>/ with lib/, include/ and MANIFEST.json (sources, toolchain, sha256 of every file),
the contents of one release archive.

    python3 tools/build-binaries.py --hakoniwa-prefix <Hakoniwa install> --mujoco-root <MuJoCo>
"""
import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

LIBRARIES = [
    "libhakoniwa_mujoco_sensors_adapters.a",      # joint, force-torque, LiDAR, rangefinder, depth and RGB cameras
    "libhakoniwa_drone_pro_sensor_adapters.a",    # IMU, magnetometer, barometer, GNSS (published as libhakoniwa_imu_gnss_sensor_adapters.a)
    "libhakoniwa_drone_pro_sensor_models.a",      # (published as libhakoniwa_imu_gnss_sensor_models.a)
    "libhakoniwa_mujoco_sensors_mujoco.a",        # MuJoCo ray caster, wrench and joint sources
    "libhakoniwa_mujoco_sensors_renderer.a",      # MuJoCo image renderer (cameras)
    "hakoniwa_mujoco_sensors_plugin.so",          # external sensor plugin (C boundary)
]

# Build names that the archive publishes under another name.
PUBLISHED_NAMES = {"libhakoniwa_drone_pro_sensor_adapters.a": "libhakoniwa_imu_gnss_sensor_adapters.a",
                   "libhakoniwa_drone_pro_sensor_models.a": "libhakoniwa_imu_gnss_sensor_models.a"}


def run(cmd, **kw):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def git(checkout, *args):
    return subprocess.run(["git", "-C", str(checkout), *args], check=True,
                          capture_output=True, text=True).stdout.strip()


def platform_name():
    system = {"Darwin": "macos", "Linux": "linux", "Windows": "windows"}[platform.system()]
    machine = platform.machine().lower().replace("x86_64", "x64").replace("amd64", "x64").replace("aarch64", "arm64")
    return f"{system}-{machine}"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hakoniwa-prefix", required=True, help="Hakoniwa (core) install prefix, for the plugin")
    ap.add_argument("--mujoco-root", required=True, help="MuJoCo release directory (include/, lib/)")
    ap.add_argument("--mujoco-library", help="MuJoCo library file (default: found under --mujoco-root/lib)")
    ap.add_argument("--build-dir", default=str(ROOT / "build/binaries"))
    args = ap.parse_args()

    sources = {n: s for n, s in json.loads((ROOT / "config/sources.json").read_text())["sources"].items()
               if n in ("sensors-pro", "drone-pro", "sensor-adapter")}
    checkouts = {}
    for name, s in sources.items():
        checkout = (ROOT / s["checkout"]).resolve()
        head = git(checkout, "rev-parse", "HEAD")
        if head != s["revision"]:
            sys.exit(f"{name}: {checkout} is at {head}, config/sources.json says {s['revision']}")
        if git(checkout, "status", "--porcelain", "--untracked-files=no"):
            sys.exit(f"{name}: {checkout} has changes to tracked files")
        checkouts[name] = checkout

    mujoco_root = Path(args.mujoco_root).resolve()
    mujoco_version = (ROOT / "MUJOCO_VERSION.txt").read_text().strip()
    mujoco_lib = Path(args.mujoco_library) if args.mujoco_library else next(
        p for p in sorted((mujoco_root / "lib").glob("libmujoco*")) if mujoco_version in p.name)
    if mujoco_version not in mujoco_lib.name:
        sys.exit(f"MuJoCo library {mujoco_lib.name} is not version {mujoco_version} (MUJOCO_VERSION.txt)")

    build = Path(args.build_dir).resolve()
    if build.exists():
        shutil.rmtree(build)
    run(["cmake", "-S", checkouts["sensors-pro"], "-B", build,
         "-DCMAKE_BUILD_TYPE=Release",
         "-DHAKO_MUJOCO_SENSORS_BUILD_TESTS=OFF",
         "-DHAKO_SENSORS_WITH_DRONE_PRO=ON",
         f"-DHAKONIWA_DRONE_PRO_ROOT={checkouts['drone-pro']}",
         f"-DHAKONIWA_SENSOR_ADAPTER_ROOT={checkouts['sensor-adapter']}",
         f"-DHAKONIWA_INSTALL_PREFIX={Path(args.hakoniwa_prefix).resolve()}",
         f"-DHAKO_MUJOCO_ROOT={mujoco_root}",
         f"-DHAKO_MUJOCO_LIBRARY={mujoco_lib}"])
    run(["cmake", "--build", build, "-j8"])

    dist = ROOT / "dist" / platform_name()
    if dist.exists():
        shutil.rmtree(dist)
    (dist / "lib").mkdir(parents=True)
    for name in LIBRARIES:
        shutil.copy2(build / name, dist / "lib" / PUBLISHED_NAMES.get(name, name))
    shutil.copytree(ROOT / "include", dist / "include")

    files = sorted(p for p in dist.rglob("*") if p.is_file())
    cmake_version = subprocess.run(["cmake", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    compiler = subprocess.run(["c++", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    manifest = {
        "platform": platform_name(),
        "built": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "sources": {n: {"repository": s["repository"], "revision": s["revision"]} for n, s in sources.items()},
        "toolchain": {"cmake": cmake_version, "compiler": compiler, "build_type": "Release",
                      "mujoco_library": mujoco_lib.name, "mujoco_version": mujoco_version},
        "files": {str(p.relative_to(dist)): sha256(p) for p in files},
    }
    (dist / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{dist}: {len(files)} files")


if __name__ == "__main__":
    main()
