# Running the tests

This runs the tests of this repository against a released binary set on your machine and writes the test status and the requirement traceability for it. Platforms: macOS arm64, and Ubuntu 24.04 x86-64 (also under WSL2 on Windows); each step gives the macOS block first, then the Linux block.

Type the blocks in order, in a terminal, from one working directory. Everything goes into `~/hakoniwa-sensor-check`.

## 1. Tools

[Homebrew](https://brew.sh) provides the build tools and the two libraries the tests need (GoogleTest; GLFW for the image renderer):

```bash
brew install cmake googletest glfw
```

On Ubuntu 24.04 (also WSL2):

```bash
sudo apt-get install -y build-essential cmake ninja-build python3 unzip curl libgtest-dev libglfw3-dev libgl-dev nlohmann-json3-dev libboost-dev
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
VERSION=0.2.0
NAME=hakoniwa-sensor-pro-$VERSION-macos-arm64
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip.sha256
shasum -a 256 -c $NAME.zip.sha256
unzip -q $NAME.zip
```

On Ubuntu 24.04 (also WSL2):

```bash
cd ~/hakoniwa-sensor-check
VERSION=0.2.0
NAME=hakoniwa-sensor-pro-$VERSION-linux-x64
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip
curl -L -O https://github.com/hakoniwalab/hakoniwa-sensor-pro/releases/download/v$VERSION/$NAME.zip.sha256
sha256sum -c $NAME.zip.sha256
unzip -q $NAME.zip
```

`shasum` and `sha256sum` print `<name>.zip: OK`.

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

On Ubuntu 24.04 (also WSL2), from the official Linux release (it already has `include/` and `lib/`):

```bash
cd ~/hakoniwa-sensor-check
MJ=$(cat hakoniwa-sensor-pro/MUJOCO_VERSION.txt)
curl -L -O https://github.com/google-deepmind/mujoco/releases/download/$MJ/mujoco-$MJ-linux-x86_64.tar.gz
tar -xzf mujoco-$MJ-linux-x86_64.tar.gz
```

Without this step, leave out `--mujoco-root` below: only the sensor-model tests run.

The tests of the external sensor plugin also need the Hakoniwa core libraries the plugin links to (`libassets`), from a Hakoniwa core install; add `--hakoniwa-prefix <install prefix>` to the run below. The core must have every function listed in the archive's `MANIFEST.json` under `runtime_dependencies` (older cores lack `hako_asset_simulation_state`); the core installed by Hakoniwa Business Pack (its `work/foundation/install`) has them. Only the library file is used: no Hakoniwa process has to run. The plugin carries no library search path of its own: the test, like a host, supplies the Hakoniwa, MuJoCo and GLFW library directories. Without the option the plugin tests are not built.

## 5. Run

```bash
cd ~/hakoniwa-sensor-check/hakoniwa-sensor-pro
python3 tools/run-tests.py --dist ../$NAME --mujoco-root ../mujoco-$(cat MUJOCO_VERSION.txt)
```

`$NAME` is the archive name from step 3 (macOS or Linux).

It first checks every file of the binary set against its `MANIFEST.json` (SHA-256) and stops if one differs, is missing or is extra:

```text
binary set ../hakoniwa-sensor-pro-0.2.0-macos-arm64: every file matches MANIFEST.json (sha256 ...)
```

Then it builds the tests in `build/tests-<platform>/` (`macos-arm64` or `linux-x64`) against the binaries and the interface only, runs them one at a time, and ends with a line `N of N passed`, N being the number of tests (the README's "Test status" gives the number for the released binary set).

The exit status is 0 when every test passed and ctest succeeded, and 1 otherwise.

## 6. What it writes

Everything goes under `build/tests-<platform>/report/`; the repository's own files are not changed.

| File | Contents |
|---|---|
| `results.json` | every test with its result, the totals by sensor and stage, ctest's exit status, and the binary set it ran on: source revisions, toolchain and the SHA-256 of its `MANIFEST.json` |
| `README-status.md` | the test status table, in the form of the README's "Test status" |
| `traceability.md` | every requirement of the specification, the tests that check it, and the result of this run |

The full test log is `build/tests-<platform>/Testing/Temporary/LastTest.log`, and the JUnit file `build/tests-<platform>/results.xml`. Under WSL2 the image-renderer tests use WSLg's display. A failed test is listed at the end of the run and in `results.json` under `failed`.

Compare `README-status.md` and `traceability.md` with the README and [traceability.md](traceability.md) of this repository: on the same binary set they show the same results.
