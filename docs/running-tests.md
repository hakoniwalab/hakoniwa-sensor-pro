# Running the tests

This runs the tests of this repository against a released binary set on your machine and writes the test status and the requirement traceability for it. Platform: macOS arm64.

Type the blocks in order, in a terminal, from one working directory. Everything goes into `~/hakoniwa-sensor-check`.

## 1. Tools

[Homebrew](https://brew.sh) provides the build tools and the two libraries the tests need (GoogleTest; GLFW for the image renderer):

```bash
brew install cmake googletest glfw
```

## 2. This repository and the two public repositories it reads

The tests build against the public interface hakoniwa-sensor-adapter at the revision the binaries were built with. The traceability reads the drone sensor test design from hakoniwa-drone-core. Both revisions are in `config/sources.json`.

```bash
mkdir -p ~/hakoniwa-sensor-check && cd ~/hakoniwa-sensor-check
git clone https://github.com/hakoniwalab/hakoniwa-sensor-pro.git
git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter.git
git clone https://github.com/toppers/hakoniwa-drone-core.git
rev() { python3 -c "import json,sys;print(json.load(open('hakoniwa-sensor-pro/config/sources.json'))['sources'][sys.argv[1]]['revision'])" "$1"; }
git -C hakoniwa-sensor-adapter checkout -q "$(rev sensor-adapter)"
git -C hakoniwa-drone-core checkout -q "$(rev drone-core)"
```

## 3. The binary set

Download the release archive and its checksum, check it, and extract it:

```bash
cd ~/hakoniwa-sensor-check
VERSION=0.1.0
NAME=hakoniwa-sensor-pro-$VERSION-macos-arm64
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip.sha256
shasum -a 256 -c $NAME.zip.sha256
unzip -q $NAME.zip
```

`shasum` prints `<name>.zip: OK`.

## 4. MuJoCo (for the tests on a MuJoCo model)

The tests on a MuJoCo model need the MuJoCo version the binaries were built with, in `hakoniwa-sensor-pro/MUJOCO_VERSION.txt` (also `toolchain.mujoco_version` in `MANIFEST.json`). This takes the library and headers out of the official macOS release:

```bash
cd ~/hakoniwa-sensor-check
MJ=$(cat hakoniwa-sensor-pro/MUJOCO_VERSION.txt)
curl -L -O https://github.com/google-deepmind/mujoco/releases/download/$MJ/mujoco-$MJ-macos-universal2.dmg
hdiutil attach mujoco-$MJ-macos-universal2.dmg -mountpoint /Volumes/MuJoCo
mkdir -p mujoco-$MJ/include/mujoco mujoco-$MJ/lib
cp -Rp /Volumes/MuJoCo/MuJoCo.app/Contents/Frameworks/MuJoCo.framework mujoco-$MJ/
hdiutil detach /Volumes/MuJoCo
cp -R mujoco-$MJ/MuJoCo.framework/Headers/* mujoco-$MJ/include/mujoco/
cp mujoco-$MJ/MuJoCo.framework/Versions/Current/libmujoco.$MJ.dylib mujoco-$MJ/lib/
```

Without this step, leave out `--mujoco-root` below: only the sensor-model tests run.

## 5. Run

```bash
cd ~/hakoniwa-sensor-check/hakoniwa-sensor-pro
python3 tools/run-tests.py --dist ../hakoniwa-sensor-pro-0.1.0-macos-arm64 --mujoco-root ../mujoco-$(cat MUJOCO_VERSION.txt)
```

It first checks every file of the binary set against its `MANIFEST.json` (SHA-256) and stops if one differs, is missing or is extra:

```text
binary set ../hakoniwa-sensor-pro-0.1.0-macos-arm64: every file matches MANIFEST.json (sha256 ...)
```

Then it builds the tests in `build/tests-macos-arm64/` against the binaries and the interface only, runs them one at a time, and ends with a line `N of N passed`, N being the number of tests (the README's "Test status" gives the number for the released binary set).

The exit status is 0 when every test passed and ctest succeeded, and 1 otherwise.

## 6. What it writes

Everything goes under `build/tests-macos-arm64/report/`; the repository's own files are not changed.

| File | Contents |
|---|---|
| `results.json` | every test with its result, the totals by sensor and stage, ctest's exit status, and the binary set it ran on: source revisions, toolchain and the SHA-256 of its `MANIFEST.json` |
| `README-status.md` | the test status table, in the form of the README's "Test status" |
| `traceability.md` | every requirement of the specification, the tests that check it, and the result of this run |

The full test log is `build/tests-macos-arm64/Testing/Temporary/LastTest.log`, and the JUnit file `build/tests-macos-arm64/results.xml`. A failed test is listed at the end of the run and in `results.json` under `failed`.

Compare `README-status.md` and `traceability.md` with the README and [traceability.md](traceability.md) of this repository: on the same binary set they show the same results.
