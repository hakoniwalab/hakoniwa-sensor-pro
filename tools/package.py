#!/usr/bin/env python3
"""Make the release archive of a binary set: hakoniwa-sensor-pro-<version>-<platform>.zip and its .sha256.

The archive holds one top directory with lib/, include/, MANIFEST.json and the license files
(LICENSE, LICENSE.md, LICENSE-ja.md, LICENSE-tests, THIRD_PARTY_NOTICES.md).

    python3 tools/package.py --version 0.2.0          # dist/<this platform>
    python3 tools/package.py --version 0.2.0 --dist dist/macos-arm64
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
verify_manifest = import_module("run-tests").verify_manifest


LICENSE_FILES = ["LICENSE", "LICENSE.md", "LICENSE-ja.md", "LICENSE-tests", "THIRD_PARTY_NOTICES.md"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True)
    ap.add_argument("--dist", default=str(ROOT / "dist" / platform_name()))
    args = ap.parse_args()

    dist = Path(args.dist).resolve()
    # The archive is made only from a binary set that matches its MANIFEST.json and whose test record
    # (results/<platform>.json, written by run-tests.py --update-repository) is for that same MANIFEST
    # and passed completely.
    manifest, manifest_sha256 = verify_manifest(dist)
    record_path = ROOT / "results" / f"{manifest['platform']}.json"
    if not record_path.exists():
        sys.exit(f"no test record {record_path}; run tools/run-tests.py --update-repository on this binary set")
    record = json.loads(record_path.read_text())
    if record["binaries"].get("manifest_sha256") != manifest_sha256:
        sys.exit(f"{record_path.name} is for another binary set (MANIFEST.json sha256 "
                 f"{record['binaries'].get('manifest_sha256')} != {manifest_sha256}); test this one first")
    if record["passed"] != record["total"] or record["total"] == 0 or record.get("ctest_exit") != 0 or record["failed"]:
        sys.exit(f"{record_path.name}: {record['passed']} of {record['total']} passed, ctest exit "
                 f"{record.get('ctest_exit')}; only a fully passed binary set is packaged")
    print(f"binary set matches MANIFEST.json and its test record: {record['passed']} of {record['total']} passed")
    license_files = [ROOT / n for n in LICENSE_FILES]
    missing = [f.name for f in license_files if not f.exists()]
    if missing:
        sys.exit(f"license files missing: {missing}")
    name = f"hakoniwa-sensor-pro-{args.version}-{manifest['platform']}"
    out = ROOT / "dist" / f"{name}.zip"
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted(dist.rglob("*")):
            if p.is_file():
                z.write(p, f"{name}/{p.relative_to(dist)}")
        for f in license_files:
            z.write(f, f"{name}/{f.name}")
    digest = hashlib.sha256(out.read_bytes()).hexdigest()
    (ROOT / "dist" / f"{name}.zip.sha256").write_text(f"{digest}  {out.name}\n")
    print(f"{out} ({out.stat().st_size} bytes)\nsha256 {digest}")


if __name__ == "__main__":
    main()
