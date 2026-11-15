#!/usr/bin/env python3
"""Copy the tests and headers listed in config/sync.json from the private sources.

Files are read from the git objects at the revisions in config/sources.json
(git show / git ls-tree), so uncommitted changes in the checkouts are never copied.
Every copied file is listed with its source and revision in config/synced-files.json.

    python3 tools/sync.py          # copy
    python3 tools/sync.py --check  # report files that differ from the sources; exit 1 if any
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def git(checkout, *args):
    return subprocess.run(["git", "-C", str(checkout), *args], check=True, capture_output=True).stdout


def entries(checkout, revision, path):
    """(source path, relative path) for a file or every file under a directory."""
    if not path.endswith("/"):
        return [(path, "")]
    out = git(checkout, "ls-tree", "-r", "--name-only", revision, path).decode().splitlines()
    return [(p, p[len(path):]) for p in out]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    sources = json.loads((ROOT / "config/sources.json").read_text())["sources"]
    plan = json.loads((ROOT / "config/sync.json").read_text())["copy"]

    for name, s in sources.items():
        checkout = (ROOT / s["checkout"]).resolve()
        if not (checkout / ".git").exists():
            sys.exit(f"{name}: no checkout at {checkout}")
        git(checkout, "cat-file", "-e", s["revision"] + "^{commit}")

    listed, differ = [], []
    for item in plan:
        s = sources[item["source"]]
        checkout = (ROOT / s["checkout"]).resolve()
        found = entries(checkout, s["revision"], item["from"])
        if not found:
            sys.exit(f"nothing at {item['source']}:{item['from']} ({s['revision']})")
        for src, rel in found:
            dest = ROOT / (item["to"] + rel if item["to"].endswith("/") else item["to"])
            data = git(checkout, "show", f"{s['revision']}:{src}")
            listed.append({"path": str(dest.relative_to(ROOT)), "source": item["source"],
                           "from": src, "revision": s["revision"]})
            if dest.exists() and dest.read_bytes() == data:
                continue
            differ.append(str(dest.relative_to(ROOT)))
            if not args.check:
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)

    if args.check:
        for d in differ:
            print("differs:", d)
        sys.exit(1 if differ else 0)
    listed.sort(key=lambda e: e["path"])
    (ROOT / "config/synced-files.json").write_text(json.dumps(
        {"description": "Written by tools/sync.py. Do not edit.", "files": listed}, indent=2) + "\n")
    print(f"{len(listed)} files, {len(differ)} updated")


if __name__ == "__main__":
    main()
