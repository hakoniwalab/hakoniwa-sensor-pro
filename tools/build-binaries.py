#!/usr/bin/env python3
"""Build the binary set from the private sources at the revisions in config/sources.json.

The sources are exported from the checkouts' git objects at those revisions (git archive), so the
checkouts may be on any branch and need not be clean. The result is
dist/<platform>/ with lib/, include/ and MANIFEST.json (sources, toolchain, sha256 of every file),
the contents of one release archive.

    python3 tools/build-binaries.py --hakoniwa-prefix <Hakoniwa install> --mujoco-root <MuJoCo>
"""
import argparse
import hashlib
import json
import os
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


def strip_build_paths(plugin):
    """The shipped plugin must not carry this machine's library paths: no rpath, and its libraries found by name.

    macOS: delete every LC_RPATH, refer to GLFW as @rpath/libglfw.3.dylib, re-sign (ad hoc).
    Linux: CMAKE_SKIP_RPATH leaves no RPATH/RUNPATH; checked here.
    """
    if platform.system() == "Darwin":
        out = subprocess.run(["otool", "-l", str(plugin)], check=True, capture_output=True, text=True).stdout
        lines = out.splitlines()
        rpaths = [lines[i + 2].split()[1] for i, l in enumerate(lines) if "LC_RPATH" in l]
        for r in rpaths:
            run(["install_name_tool", "-delete_rpath", r, plugin])
        deps = subprocess.run(["otool", "-L", str(plugin)], check=True, capture_output=True, text=True).stdout
        for line in deps.splitlines()[1:]:
            path = line.strip().split(" (")[0]
            if "libglfw" in path and not path.startswith("@rpath/"):
                run(["install_name_tool", "-change", path, "@rpath/" + Path(path).name, plugin])
        run(["codesign", "--force", "--sign", "-", plugin])
    elif platform.system() == "Linux":
        out = subprocess.run(["readelf", "-d", str(plugin)], check=True, capture_output=True, text=True).stdout
        if "RPATH" in out or "RUNPATH" in out:
            sys.exit(f"{plugin} carries an RPATH/RUNPATH: " + " ".join(l for l in out.splitlines() if "PATH" in l))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plugin_dependencies(plugin):
    """What the plugin needs at load time, read from the binary: the shared libraries it names and the Hakoniwa
    core functions it imports. PDU and services call the core only when the host passes pdu_config_path or
    service_config_dir; the library must still be found to load the plugin."""
    if sys.platform == "darwin":
        lines = subprocess.run(["otool", "-L", str(plugin)], capture_output=True, text=True, check=True).stdout.splitlines()[1:]
        needed = [line.strip().split(" (")[0] for line in lines if line.strip()]
    else:
        out = subprocess.run(["readelf", "-d", str(plugin)], capture_output=True, text=True, check=True).stdout
        needed = [line.split("[", 1)[1].rstrip("]") for line in out.splitlines() if "(NEEDED)" in line]
    undefined = subprocess.run(["nm", "-u", str(plugin)], capture_output=True, text=True, check=True).stdout.split()
    hako = sorted({u.lstrip("_").split("@")[0] for u in undefined if u.lstrip("_").startswith("hako_")})
    return {
        "shared_libraries": needed,
        "hakoniwa_core_functions": hako,
        "notes": [
            "Loading the plugin needs the Hakoniwa core library (libassets) whose functions are listed here, "
            "MuJoCo of toolchain.mujoco_version, GLFW 3.3 or later and OpenGL; the plugin carries no search path, "
            "the host supplies the library directories.",
            "PDU output and services call the Hakoniwa core only when the host passes pdu_config_path or "
            "service_config_dir; without them no core function is called.",
            "The static libraries in lib/ need no Hakoniwa core; the MuJoCo parts need MuJoCo.",
        ],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hakoniwa-prefix", required=True, help="Hakoniwa (core) install prefix, for the plugin")
    ap.add_argument("--mujoco-root", required=True, help="MuJoCo release directory (include/, lib/)")
    ap.add_argument("--mujoco-library", help="MuJoCo library file (default: found under --mujoco-root/lib)")
    ap.add_argument("--build-dir", default=str(ROOT / "build/binaries"))
    args = ap.parse_args()

    sources = {n: s for n, s in json.loads((ROOT / "config/sources.json").read_text())["sources"].items()
               if n in ("sensors-pro", "drone-pro", "sensor-adapter")}
    # The sources are exported from the git objects at the pinned revisions into an empty directory
    # (git archive), so neither uncommitted changes nor untracked files of the checkouts reach the build.
    build = Path(args.build_dir).resolve()
    if build.exists():
        shutil.rmtree(build)
    exported = {}
    for name, s in sources.items():
        checkout = (ROOT / s["checkout"]).resolve()
        git(checkout, "cat-file", "-e", s["revision"] + "^{commit}")
        target = build / "sources" / name
        target.mkdir(parents=True)
        archive = subprocess.run(["git", "-C", str(checkout), "archive", s["revision"]], check=True, capture_output=True).stdout
        subprocess.run(["tar", "-x", "-C", str(target)], input=archive, check=True)
        exported[name] = target
        print(f"{name}: {s['revision']} -> {target}")

    # The copied tests and headers must be those of the same revisions.
    if subprocess.run([sys.executable, str(ROOT / "tools/sync.py"), "--check"]).returncode != 0:
        sys.exit("files copied by tools/sync.py differ from the sources at the pinned revisions; run tools/sync.py")

    mujoco_root = Path(args.mujoco_root).resolve()
    mujoco_version = (ROOT / "MUJOCO_VERSION.txt").read_text().strip()
    mujoco_lib = Path(args.mujoco_library) if args.mujoco_library else next(
        p for p in sorted((mujoco_root / "lib").glob("libmujoco*")) if mujoco_version in p.name)
    if mujoco_version not in mujoco_lib.name:
        sys.exit(f"MuJoCo library {mujoco_lib.name} is not version {mujoco_version} (MUJOCO_VERSION.txt)")

    # git inside the exported sources must not find this repository above them.
    os.environ["GIT_CEILING_DIRECTORIES"] = str(build / "sources")
    run(["cmake", "-S", exported["sensors-pro"], "-B", build / "cmake",
         "-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_SKIP_RPATH=ON",  # the host supplies the Hakoniwa, MuJoCo and GLFW libraries
         "-DHAKO_MUJOCO_SENSORS_BUILD_TESTS=OFF",
         "-DHAKO_SENSORS_WITH_DRONE_PRO=ON",
         f"-DHAKONIWA_DRONE_PRO_ROOT={exported['drone-pro']}",
         f"-DHAKONIWA_SENSOR_ADAPTER_ROOT={exported['sensor-adapter']}",
         f"-DHAKONIWA_INSTALL_PREFIX={Path(args.hakoniwa_prefix).resolve()}",
         f"-DHAKO_MUJOCO_ROOT={mujoco_root}",
         f"-DHAKO_MUJOCO_LIBRARY={mujoco_lib}"])
    run(["cmake", "--build", build / "cmake", "-j8"])

    dist = ROOT / "dist" / platform_name()
    if dist.exists():
        shutil.rmtree(dist)
    (dist / "lib").mkdir(parents=True)
    for name in LIBRARIES:
        shutil.copy2(build / "cmake" / name, dist / "lib" / PUBLISHED_NAMES.get(name, name))
    strip_build_paths(dist / "lib" / "hakoniwa_mujoco_sensors_plugin.so")
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
        "runtime_dependencies": {"lib/hakoniwa_mujoco_sensors_plugin.so":
                                 plugin_dependencies(dist / "lib" / "hakoniwa_mujoco_sensors_plugin.so")},
        "files": {str(p.relative_to(dist)): sha256(p) for p in files},
    }
    (dist / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{dist}: {len(files)} files")


if __name__ == "__main__":
    main()
