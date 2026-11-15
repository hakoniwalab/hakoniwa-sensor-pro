#!/usr/bin/env python3
"""Build the tests against a binary set, run them, record the result and update the README.

    python3 tools/run-tests.py                     # dist/<this platform>
    python3 tools/run-tests.py --dist <binary set> # for example an extracted release archive

Writes the results, the test status and the traceability table under build/tests-<platform>/report/.
With --update-repository (maintainers) it writes results/<platform>.json, docs/traceability.md and
the status blocks of README.md instead. The tests run one at a time (ctest -j1).
"""
import argparse
import hashlib
import json
import platform
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import traceability  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
BEGIN, END = "<!-- test-status:begin -->", "<!-- test-status:end -->"

SENSORS = ["Joint", "Force-torque", "IMU", "Magnetometer", "Barometer", "GNSS",
           "2D/3D LiDAR", "Rangefinder", "Depth camera", "RGB camera", "Common"]
STAGES = [("model", "Sensor model"), ("mujoco", "On a MuJoCo model"),
          ("plugin", "External plugin")]

# Test suite (or ctest name) -> (sensors, stage).
SUITE = {
    "JointSensor": ("Joint", "model"), "ForceTorque": ("Force-torque", "model"),
    "Lidar": ("2D/3D LiDAR", "model"), "Rangefinder": ("Rangefinder", "model"),
    "DepthCamera": ("Depth camera", "model"), "RgbCamera": ("RGB camera", "model"),
    "Common": ("Common", "model"), "ImplementationDescriptor": ("Common", "model"),
    "lidar_adapter": ("2D/3D LiDAR", "model"), "rangefinder_adapter": ("Rangefinder", "model"),
    "depth_camera_adapter": ("Depth camera", "model"), "rgb_camera_adapter": ("RGB camera", "model"),
    "hakoniwa_imu_sensor_adapter": ("IMU", "model"),
    "hakoniwa_mag_baro_gps_sensor_adapters": ("Magnetometer, Barometer, GNSS", "model"),
    "MujocoRayCaster": ("Common", "mujoco"), "MujocoMount": ("Common", "mujoco"),
    "MujocoImageRenderer": ("Common", "mujoco"),
    "MujocoForceTorque": ("Force-torque", "mujoco"), "MujocoJointSensor": ("Joint", "mujoco"),
    "MujocoRangefinder": ("Rangefinder", "mujoco"), "MujocoLidar": ("2D/3D LiDAR", "mujoco"),
    "MujocoDepthCamera": ("Depth camera", "mujoco"), "MujocoDepthRender": ("Depth camera", "mujoco"),
    "mujoco_rangefinder": ("Rangefinder", "mujoco"), "mujoco_lidar": ("2D/3D LiDAR", "mujoco"),
    "mujoco_depth_camera": ("Depth camera", "mujoco"), "mujoco_rgb_camera": ("RGB camera, Depth camera", "mujoco"),
    "no_physics_engine": ("Common", "model"), "no_build_paths": ("Common", "model"),
    "PluginJoint": ("Joint", "plugin"), "PluginForceTorque": ("Force-torque", "plugin"),
    "PluginSpec": ("Common", "plugin"),
    "PublicForceTorqueSpec": ("Force-torque", "model"), "PublicJointSpec": ("Joint", "model"),
}
# Test ID T_<code>_<U|I>...: the code names the sensor; U is a unit test of the model,
# I an integration test (on MuJoCo, or through the plugin for PLG).
ID = {"ACC": "IMU", "GYR": "IMU", "MAG": "Magnetometer", "BAR": "Barometer", "GPS": "GNSS",
      "LID": "2D/3D LiDAR", "RNG": "Rangefinder", "DEP": "Depth camera", "RGB": "RGB camera",
      "COM": "Common", "MJR": "Common", "MJI": "Common", "PLG": "Common"}

# How the README names the sources of a binary set.
SOURCE_LABELS = {"sensors-pro": "implementation", "drone-pro": "IMU and GNSS models",
                 "sensor-adapter": "interface (hakoniwa-sensor-adapter)"}


def platform_name():
    system = {"Darwin": "macos", "Linux": "linux", "Windows": "windows"}[platform.system()]
    machine = platform.machine().lower().replace("x86_64", "x64").replace("amd64", "x64").replace("aarch64", "arm64")
    return f"{system}-{machine}"


def classify(name):
    """(sensors, stage) of a test."""
    suite, _, test = name.partition(".")
    if suite == "PublicImuGnssSpec" and not test.startswith("T_"):
        for word, sensor in (("Gyro", "IMU"), ("Accel", "IMU"), ("Magnetometer", "Magnetometer"),
                             ("Baro", "Barometer"), ("Gps", "GNSS")):
            if test.startswith(word):
                return [sensor], "model"
    m = re.match(r"T_([A-Z]{3})_([UI])", test)
    if m and m.group(1) in ("ACC", "GYR", "MAG", "BAR", "GPS") or (m and m.group(1) == "COM"
                                                                     and suite in ("SensorModels", "PublicImuGnssSpec")):
        # The IMU, magnetometer, barometer and GNSS tests use no MuJoCo, whatever their stage letter.
        return [ID[m.group(1)]], "model"
    if m and m.group(1) in ID:
        stage = "model" if m.group(2) == "U" else ("plugin" if m.group(1) == "PLG" or suite.startswith("Plugin") else "mujoco")
        sensor = ID[m.group(1)]
        if suite.startswith("Plugin") and suite[6:] in ("Joint", "ForceTorque"):
            sensor = {"Joint": "Joint", "ForceTorque": "Force-torque"}[suite[6:]]
        return [sensor], stage
    if suite in SUITE:
        sensors, stage = SUITE[suite]
        return [x.strip() for x in sensors.split(",")], stage
    if suite.startswith("Plugin"):
        return ["Common"], "plugin"
    return ["Common"], "model"


def verify_manifest(dist):
    """Every file of the binary set matches MANIFEST.json, and no file is missing or extra."""
    manifest = json.loads((dist / "MANIFEST.json").read_text())
    listed = manifest["files"]
    present = {str(p.relative_to(dist)) for p in dist.rglob("*") if p.is_file()} - {"MANIFEST.json", "LICENSE", "LICENSE.md", "LICENSE-ja.md", "LICENSE-tests", "THIRD_PARTY_NOTICES.md"}
    errors = [f"missing: {f}" for f in sorted(set(listed) - present)]
    errors += [f"not in MANIFEST.json: {f}" for f in sorted(present - set(listed))]
    for f in sorted(set(listed) & present):
        if hashlib.sha256((dist / f).read_bytes()).hexdigest() != listed[f]:
            errors.append(f"SHA-256 differs: {f}")
    if errors:
        sys.exit("the binary set does not match its MANIFEST.json:\n  " + "\n  ".join(errors))
    return manifest, hashlib.sha256((dist / "MANIFEST.json").read_bytes()).hexdigest()


def run(cmd):
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True)


def render(results):
    out = [BEGIN, ""]
    for r in results:
        out += [f"### {r['platform']}", "",
                f"- Run: {r['run']} — {r['passed']} of {r['total']} tests passed",
                f"- Binary set built {r['binaries']['built']} from "
                + ", ".join(f"{SOURCE_LABELS.get(n, n)} `{s['revision'][:8]}`" for n, s in r["binaries"]["sources"].items()),
                f"- Toolchain: {r['binaries']['toolchain']['compiler']}",
                f"- Binary set: MANIFEST.json sha256 `{r['binaries'].get('manifest_sha256', '?')}` (every file checked against it before the run)", ""]
        out += ["| Sensor | " + " | ".join(t for _, t in STAGES) + " |", "|---|" + "---|" * len(STAGES)]
        for sensor in SENSORS:
            st = r["by_sensor"].get(sensor)
            if not st:
                continue
            cells = []
            for key, _ in STAGES:
                c = st.get(key)
                cells.append(f"{c['passed']} / {c['total']}" if c else "—")
            out.append(f"| {sensor} | " + " | ".join(cells) + " |")
        out += ["", "Passed / run. Common: the implementation descriptor, the MuJoCo ray caster and image renderer, and mounting."]
        if r["failed"]:
            out += ["", "Failed: " + ", ".join(f"`{f}`" for f in r["failed"])]
        out.append("")
    out.append(END)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", default=str(ROOT / "dist" / platform_name()))
    ap.add_argument("--mujoco-root", help="MuJoCo release directory: adds the tests on a MuJoCo model")
    ap.add_argument("--mujoco-library", help="MuJoCo library file")
    ap.add_argument("--hakoniwa-prefix", help="Hakoniwa core install prefix: adds the external plugin tests "
                    "(with --mujoco-root)")
    ap.add_argument("--update-repository", action="store_true",
                    help="maintainers: write results/<platform>.json, docs/traceability.md and the README status "
                         "(default: write them under build/tests-<platform>/report/ only)")
    args = ap.parse_args()
    # Cleared unless given, so an old cache cannot linger.
    extra = ["-DHAKO_MUJOCO_ROOT=", "-DHAKO_MUJOCO_LIBRARY=",
             f"-DHAKONIWA_INSTALL_PREFIX={Path(args.hakoniwa_prefix).resolve() if args.hakoniwa_prefix else ''}"]
    if args.mujoco_root:
        version = (ROOT / "MUJOCO_VERSION.txt").read_text().strip()
        lib = args.mujoco_library or next(str(p) for p in sorted(Path(args.mujoco_root, "lib").glob("libmujoco*"))
                                          if version in p.name)
        if version not in Path(lib).name:
            sys.exit(f"MuJoCo library {lib} is not version {version} (MUJOCO_VERSION.txt)")
        extra += [f"-DHAKO_MUJOCO_ROOT={Path(args.mujoco_root).resolve()}", f"-DHAKO_MUJOCO_LIBRARY={lib}"]

    dist = Path(args.dist).resolve()
    manifest, manifest_sha256 = verify_manifest(dist)
    print(f"binary set {dist}: every file matches MANIFEST.json (sha256 {manifest_sha256})")

    build = ROOT / "build" / f"tests-{manifest['platform']}"
    run(["cmake", "-S", ROOT / "tests", "-B", build, "-DCMAKE_BUILD_TYPE=Release", f"-DHAKO_SENSOR_PRO_DIST={dist}", *extra])
    run(["cmake", "--build", build, "-j8"])
    junit = build / "results.xml"
    if junit.exists():
        junit.unlink()
    ctest_exit = subprocess.run(["ctest", "-j1", "--output-junit", str(junit)], cwd=build).returncode
    if not junit.exists():
        sys.exit(f"ctest wrote no results (exit {ctest_exit})")

    cases = ET.parse(junit).getroot().iter("testcase")
    by_sensor, failed, total, passed, per_test = {}, [], 0, 0, {}
    for c in cases:
        name = c.get("name")
        ok = c.get("status") == "run" and c.find("failure") is None and c.find("error") is None
        total += 1
        passed += ok
        per_test[name] = ok
        if not ok:
            failed.append(name)
        sensors, stage = classify(name)
        for s in sensors:
            t = by_sensor.setdefault(s, {}).setdefault(stage, {"total": 0, "passed": 0})
            t["total"] += 1
            t["passed"] += ok

    result = {"platform": manifest["platform"],
              "run": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
              "total": total, "passed": passed, "failed": failed,
              "ctest_exit": ctest_exit,
              "binaries": {**{k: manifest[k] for k in ("built", "sources", "toolchain")},
                           "manifest_sha256": manifest_sha256},
              "by_sensor": by_sensor, "tests": per_test}
    if not args.update_repository:
        report = build / "report"
        report.mkdir(exist_ok=True)
        (report / "results.json").write_text(json.dumps(result, indent=2) + "\n")
        (report / "README-status.md").write_text(render([result]) + "\n")
        traceability.update([result], out_dir=report)
        print(f"report: {report}")
    else:
        (ROOT / "results").mkdir(exist_ok=True)
        (ROOT / "results" / f"{manifest['platform']}.json").write_text(json.dumps(result, indent=2) + "\n")
        results = [json.loads(p.read_text()) for p in sorted((ROOT / "results").glob("*.json"))]
        readme = (ROOT / "README.md").read_text()
        if BEGIN not in readme:
            sys.exit("README.md has no test-status markers")
        head, rest = readme.split(BEGIN, 1)
        tail = rest.split(END, 1)[1]
        (ROOT / "README.md").write_text(head + render(results) + tail)
        traceability.update(results)
    print(f"{passed} of {total} passed")
    ok = passed == total and total > 0 and ctest_exit == 0
    if not ok and ctest_exit != 0 and passed == total:
        print(f"ctest exited {ctest_exit} although every recorded test passed")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
