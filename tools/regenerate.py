#!/usr/bin/env python3
"""Rewrite the generated parts of the repository from the recorded results, without running tests.

The test status in README.md and docs/traceability.md (with its summary in README.md) are written from
results/*.json. CI runs this and fails when the committed files differ from what it writes.

    python3 tools/regenerate.py           # rewrite
    python3 tools/regenerate.py --check   # report a difference and exit 1
"""
import argparse
import json
import subprocess
import sys
from importlib import import_module
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
run_tests = import_module("run-tests")
traceability = import_module("traceability")

GENERATED = ["README.md", "docs/traceability.md"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    before = {f: (ROOT / f).read_text() for f in GENERATED}

    results = [json.loads(p.read_text()) for p in sorted((ROOT / "results").glob("*.json"))]
    readme = (ROOT / "README.md").read_text()
    head, rest = readme.split(run_tests.BEGIN, 1)
    (ROOT / "README.md").write_text(head + run_tests.render(results) + rest.split(run_tests.END, 1)[1])
    traceability.update(results)

    changed = [f for f in GENERATED if (ROOT / f).read_text() != before[f]]
    if args.check:
        for f in GENERATED:
            (ROOT / f).write_text(before[f])
        if changed:
            print("generated files differ from results/*.json: " + ", ".join(changed))
            print("run python3 tools/regenerate.py and commit")
            return 1
        print("generated files match results/*.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
