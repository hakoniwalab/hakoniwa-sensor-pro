#!/usr/bin/env python3
"""The sensor models do not depend on a physics engine or a renderer.

[RIS-COM-030] [FTS-ADP-010] [JNT-ADP-010]

Checks the released binary set, not the sources:
1. The sensor-model libraries reference no MuJoCo, OpenGL or GLFW symbol (`nm -u`).
2. The public headers of the sensor models include no MuJoCo, OpenGL or GLFW header.
The MuJoCo parts (ray caster, image renderer) and the plugin are separate libraries and are
not checked here: they are the host side that gives the models the world.

    python3 no_physics_engine.py <binary set>
"""
import re
import subprocess
import sys
from pathlib import Path

MODEL_LIBRARIES = ["libhakoniwa_mujoco_sensors_adapters.a",
                   "libhakoniwa_imu_gnss_sensor_adapters.a",
                   "libhakoniwa_imu_gnss_sensor_models.a"]
MODEL_HEADERS = ["rangefinder_sensor_adapter.hpp", "lidar_sensor_adapter.hpp",
                 "depth_camera_sensor_adapter.hpp", "rgb_camera_sensor_adapter.hpp",
                 "force_torque_sensor_adapter.hpp", "joint_sensor_adapter.hpp"]
# MuJoCo (mj_, mju_, mjv_, mjr_ ...), OpenGL (gl + capital), GLFW.
SYMBOL = re.compile(r"^_?(mj[a-z]*_\w+|gl[A-Z]\w*|glfw\w*)$")
HEADER = re.compile(r'#\s*include\s*[<"](mujoco/|GL/|OpenGL/|GLFW/)')


def main():
    dist = Path(sys.argv[1])
    failures = 0
    for name in MODEL_LIBRARIES:
        out = subprocess.run(["nm", "-u", str(dist / "lib" / name)], check=True,
                             capture_output=True, text=True).stdout
        found = sorted({s for s in (line.split()[-1] for line in out.splitlines() if line.split())
                        if SYMBOL.match(s)})
        print(f"{name}: {len(found)} physics-engine or renderer symbols")
        for s in found:
            print(f"  FAIL: {s}")
        failures += len(found)
    for name in MODEL_HEADERS:
        text = (dist / "include/hakoniwa/mujoco_sensors" / name).read_text()
        found = HEADER.findall(text)
        print(f"{name}: {len(found)} physics-engine or renderer includes")
        failures += len(found)
    print("PASS" if failures == 0 else f"FAIL: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
