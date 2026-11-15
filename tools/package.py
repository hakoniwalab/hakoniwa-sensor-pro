#!/usr/bin/env python3
"""Make the release archive of a binary set: hakoniwa-sensor-pro-<version>-<platform>.zip and its .sha256.

The archive holds one top directory with lib/, include/, MANIFEST.json and LICENSE.

    python3 tools/package.py --version 0.1.0          # dist/<this platform>
    python3 tools/package.py --version 0.1.0 --dist dist/macos-arm64
"""
import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
from importlib import import_module  # noqa: E402

platform_name = import_module("build-binaries").platform_name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True)
    ap.add_argument("--dist", default=str(ROOT / "dist" / platform_name()))
    ap.add_argument("--allow-no-license", action="store_true", help="for trying the packaging before LICENSE exists")
    args = ap.parse_args()

    dist = Path(args.dist).resolve()
    manifest = json.loads((dist / "MANIFEST.json").read_text())
    license_file = ROOT / "LICENSE"
    if not license_file.exists() and not args.allow_no_license:
        sys.exit("LICENSE is missing")
    name = f"hakoniwa-sensor-pro-{args.version}-{manifest['platform']}"
    out = ROOT / "dist" / f"{name}.zip"
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(dist.rglob("*")):
            if p.is_file():
                z.write(p, f"{name}/{p.relative_to(dist)}")
        if license_file.exists():
            z.write(license_file, f"{name}/LICENSE")
    digest = hashlib.sha256(out.read_bytes()).hexdigest()
    (ROOT / "dist" / f"{name}.zip.sha256").write_text(f"{digest}  {out.name}\n")
    print(f"{out} ({out.stat().st_size} bytes)\nsha256 {digest}")


if __name__ == "__main__":
    main()
