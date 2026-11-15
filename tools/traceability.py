#!/usr/bin/env python3
"""Requirement -> test -> result, from the interface specification and the test sources.

The requirements are read from hakoniwa-sensor-adapter's docs/spec at the revision in
config/sources.json. A test names the requirements it checks in its comments, as
[FTS-ADP-120] or RIS-LID-090; a range FTS-ADP-310..340 names every requirement between.
In a GoogleTest file, the requirements belong to the TEST whose block (with the comment
just above it) they are in; a file without TEST is one test. The results are those in
results/<platform>.json. Requirements no test names are explained in
config/traceability-notes.json.

Writes docs/traceability.md and the summary between the traceability markers in README.md.
Run by tools/run-tests.py; can also be run on its own after it.
"""
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BEGIN, END = "<!-- traceability:begin -->", "<!-- traceability:end -->"

# Specification files that apply to the implementation (the others are for hosts).
SPECS = [("README.md", "Common (rangefinder, LiDAR, depth and RGB cameras)"),
         ("rangefinder.md", "Rangefinder"), ("lidar.md", "LiDAR"),
         ("depth-camera.md", "Depth camera"), ("rgb-camera.md", "RGB camera"),
         ("force-torque.md", "Force-torque sensor"), ("joint.md", "Joint sensor"),
         ("drone-common.md", "IMU, magnetometer, barometer and GNSS: common"), ("gyro.md", "Gyroscope"),
         ("accelerometer.md", "Accelerometer"), ("magnetometer.md", "Magnetometer"),
         ("barometer.md", "Barometer"), ("gps.md", "GPS")]
# Drone sensor test design (requirement -> test IDs), in hakoniwa-drone-core.
TEST_DESIGNS = ["test-design.md", "gyro/test-design.md", "acceleration/test-design.md",
                "geomagnet/test-design.md", "barometric/test-design.md", "gps/test-design.md"]
DESIGN_PATH = "docs/architecture/aircraft/sensor/"
SPEC_URL = "https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/{rev}/docs/spec/{file}"
REQ = re.compile(r"\b((?:RIS|FTS|JNT|SEN)-[A-Z]{3}-\d{3})(?:\.\.(\d{3}))?")
STATUS = {"internal": "Hakoniwa Lab internal test (publication in preparation)",
          "per-sensor": "Checked by each sensor's tests", "retired": "Retired",
          "unnamed": "Test design names a check without a test ID", "host": "Host requirement (not the adapter's)", "elsewhere": "Implementation test not in this repository",
          "review": "Code review", "private-test": "Implementation test (internal headers, not published)",
          "none": "No test"}

# Test files without TEST(...): the ctest name of the whole file.
FILE_TESTS = {"rangefinder_adapter_test.cpp": "rangefinder_adapter", "lidar_adapter_test.cpp": "lidar_adapter",
              "depth_camera_adapter_test.cpp": "depth_camera_adapter", "rgb_camera_adapter_test.cpp": "rgb_camera_adapter",
              "mujoco_rangefinder_test.cpp": "mujoco_rangefinder", "mujoco_lidar_test.cpp": "mujoco_lidar",
              "mujoco_depth_camera_test.cpp": "mujoco_depth_camera", "mujoco_rgb_camera_test.cpp": "mujoco_rgb_camera"}


def requirements():
    """[(id, spec file, text, who)] in specification order, and the specification revision."""
    s = json.loads((ROOT / "config/sources.json").read_text())["sources"]["sensor-adapter"]
    checkout = (ROOT / s["checkout"]).resolve()
    rev = s.get("specification_revision", s["revision"])
    out, seen = [], set()
    for file, _ in SPECS:
        text = subprocess.run(["git", "-C", str(checkout), "show", f"{rev}:docs/spec/{file}"],
                              check=True, capture_output=True, text=True).stdout
        for line in text.splitlines():
            m = re.match(r"^\|\s*((?:RIS|FTS|JNT|SEN)-[A-Z]{3}-\d{3})\s*\|(.*)\|\s*$", line)
            if not m or m.group(1) in seen:
                continue
            cells = [c.strip() for c in m.group(2).split("|")]
            who = cells[1] if m.group(1).startswith("SEN-") and len(cells) >= 3 else "Adapter"
            seen.add(m.group(1))
            out.append((m.group(1), file, cells[0], who))
    return out, rev


def design_tests():
    """Test ID (T-GYR-U2-020) -> requirement IDs, from the IMU, magnetometer, barometer and GNSS test design."""
    s = json.loads((ROOT / "config/sources.json").read_text())["sources"]["drone-core"]
    checkout = (ROOT / s["checkout"]).resolve()
    out = {}
    for file in TEST_DESIGNS:
        text = subprocess.run(["git", "-C", str(checkout), "show", f"{s['revision']}:{DESIGN_PATH}{file}"],
                              check=True, capture_output=True, text=True).stdout
        for line in text.splitlines():
            m = re.match(r"^\|\s*(SEN-[A-Z]{3}-\d{3})\s*\|", line)
            if m:
                cells = [c.strip() for c in line.strip().strip("|").split("|")]
                DESIGN_ROWS.setdefault(m.group(1), (cells[1] if len(cells) > 1 else "", cells[2] if len(cells) > 2 else ""))
                for t in re.findall(r"T-[A-Z]{3}-[A-Z]\d?-\d{3}", line):
                    out.setdefault(t, set()).add(m.group(1))
    return out, s["revision"]


DESIGN_ROWS = {}  # requirement -> (stages, tests) as written in the test design


def test_ids(name):
    """Test IDs a test name stands for: T_ACC_U2_010_Through_070_X -> T-ACC-U2-010 ... 070."""
    test = name.partition(".")[2] or name
    m = re.match(r"T_([A-Z]{3})_([A-Z]\d?)_(.*)", test)
    if not m:
        return []
    code, stage, ids, through = m.group(1), m.group(2), [], False
    for tok in m.group(3).split("_"):
        if re.fullmatch(r"\d{3}", tok):
            if through and ids:
                lo = int(ids[-1].rsplit("-", 1)[1])
                ids += [f"T-{code}-{stage}-{n:03d}" for n in range(lo + 10, int(tok) + 1, 10)]
            else:
                ids.append(f"T-{code}-{stage}-{tok}")
            through = False
        elif tok == "Through":
            through = True
        elif re.fullmatch(r"[UIFS]\d?", tok):
            stage = tok
        elif tok == "And":
            continue
        else:
            break
    return ids


def expand(m, known):
    first = m.group(1)
    if not m.group(2):
        return [first]
    prefix, lo = first.rsplit("-", 1)
    return [k for k in known if k.startswith(prefix + "-") and int(lo) <= int(k.rsplit("-", 1)[1]) <= int(m.group(2))]


def tests_by_requirement(known, design):
    found = {}
    paths = (sorted((ROOT / "tests/sensors/test").glob("*.cpp")) + sorted((ROOT / "tests/public").glob("*.cpp"))
             + sorted((ROOT / "tests/imu-gnss").rglob("*.cpp"))
             + sorted((ROOT / "tests/binary").glob("*.py")))
    for path in paths:
        lines = path.read_text().splitlines()
        heads = [i for i, l in enumerate(lines) if l.startswith("TEST(")]
        if not heads:
            blocks = [(FILE_TESTS.get(path.name, path.stem.replace("test_", "", 1)), "\n".join(lines))]
        else:
            starts = []
            for h in heads:  # the block starts at the comment just above TEST
                s = h
                while s > 0 and lines[s - 1].lstrip().startswith("//"):
                    s -= 1
                starts.append(s)
            blocks = []
            for k, h in enumerate(heads):
                m = re.match(r"TEST\((\w+),\s*(\w+)\)", lines[h])
                end = starts[k + 1] if k + 1 < len(starts) else len(lines)
                blocks.append((f"{m.group(1)}.{m.group(2)}", "\n".join(lines[starts[k]:end])))
        for name, text in blocks:
            for m in REQ.finditer(text):
                for r in expand(m, known):
                    found.setdefault(r, []).append(name)
            for t in test_ids(name):
                for r in design.get(t, ()):
                    found.setdefault(r, []).append(name)
    return {r: sorted(set(t)) for r, t in found.items()}


def update(results, out_dir=None):
    """results: the results/*.json of each platform, with 'tests' {ctest name: passed}."""
    reqs, rev = requirements()
    known = [r[0] for r in reqs]
    design, design_rev = design_tests()
    designed = {}
    for t, rs in design.items():
        for r in rs:
            designed.setdefault(r, []).append(t)
    tests = tests_by_requirement(known, design)
    who = {r[0]: r[3] for r in reqs}
    notes = json.loads((ROOT / "config/traceability-notes.json").read_text())["notes"]
    platforms = [r["platform"] for r in results]

    def verdict(rid):
        if rid in tests:
            cells = []
            for r in results:
                ran = [r["tests"].get(t) for t in tests[rid]]
                if any(v is None for v in ran):
                    cells.append("not run")
                else:
                    cells.append("passed" if all(ran) else "FAILED")
            return "tested", cells
        if rid in notes and notes[rid].get("tests"):
            tests[rid] = notes[rid]["tests"]
            return verdict(rid)
        if rid in notes:
            return notes[rid]["status"], ["—"] * len(results)
        if who.get(rid, "Adapter") == "Host":
            return "host", ["—"] * len(results)
        if rid in designed:
            return "elsewhere", ["—"] * len(results)
        stages, where = DESIGN_ROWS.get(rid, ("", ""))
        if re.match(r"R\b|R-", where) or stages == "R":
            return "review", ["—"] * len(results)
        if where:
            return "unnamed", ["—"] * len(results)
        return "none", ["—"] * len(results)

    lines = ["# Requirement traceability", "",
             "Generated by `tools/traceability.py` from the specification of "
             f"[hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter) at `{rev[:8]}` "
             "and the tests in `tests/`; the IMU, magnetometer, barometer and GNSS tests are linked to the requirements through the test design in "
             f"[hakoniwa-drone-core](https://github.com/toppers/hakoniwa-drone-core/tree/{design_rev}/docs/architecture/aircraft/sensor) "
             f"at `{design_rev[:8]}` (test ID in the test name). Do not edit.", ""]
    summary = []
    for file, title in SPECS:
        rows = [r for r in reqs if r[1] == file]
        if not rows:
            continue
        url = SPEC_URL.format(rev=rev, file=file)
        lines += [f"## {title}", "", f"Specification: [{file}]({url})", "",
                  "| Requirement | Specification | Tests | " + " | ".join(platforms) + " |",
                  "|---|---|---|" + "---|" * len(platforms)]
        count = {"tested": 0, "passed": 0, "internal": 0, "per-sensor": 0, "retired": 0, "unnamed": 0, "host": 0, "elsewhere": 0, "review": 0, "private-test": 0, "none": 0}
        for rid, _, text, _who in rows:
            status, cells = verdict(rid)
            count[status] += 1
            if status == "tested" and all(c == "passed" for c in cells):
                count["passed"] += 1
            if status == "tested":
                where = "<br>".join(f"`{t}`" for t in tests[rid])
            else:
                note = notes.get(rid, {}).get("note", "") or (", ".join(sorted(designed[rid])) if status == "elsewhere" else "")
                if not note and rid in DESIGN_ROWS:
                    note = f"test design: {DESIGN_ROWS[rid][1]}"
                where = STATUS[status] + (f": {note}" if note else "")
            short = text if len(text) <= 140 else text[:137] + "..."
            lines.append(f"| {rid} | {short.replace('|', '/')} | {where} | " + " | ".join(cells) + " |")
        lines.append("")
        summary.append((title, f"docs/traceability.md#{title.lower().replace(' ', '-').replace('(', '').replace(')', '').replace(',', '')}",
                        len(rows), count))
    reasons = [("retired", "retired"), ("host", "host requirement"), ("per-sensor", "checked by each sensor's tests"),
               ("internal", "Hakoniwa Lab internal test"), ("unnamed", "check without a test ID"),
               ("elsewhere", "test outside this repository"), ("review", "code review"),
               ("none", "no test")]
    table = ["| Specification | Requirements | Tested | Passed | Not tested here: reason |",
             "|---|---:|---:|---:|---|"]
    for title, link, n, c in summary:
        why = ", ".join(f"{label} {c[key]}" for key, label in reasons if c[key]) or "—"
        table.append(f"| [{title}]({link}) | {n} | {c['tested']} | {c['passed']} | {why} |")
    note = ("Passed: tested requirements whose tests all passed on every platform. "
            "The reason for each requirement not tested here is in its row below.")

    doc = (out_dir or ROOT / "docs") / "traceability.md"
    doc.parent.mkdir(parents=True, exist_ok=True)
    head_lines, body = lines[:4], lines[4:]
    summary_doc = [t.replace("](docs/traceability.md#", "](#") for t in table]
    doc.write_text("\n".join(head_lines + ["## Summary", ""] + summary_doc + ["", note, ""] + body))
    if out_dir:
        return

    out = [BEGIN, ""] + table + ["", note.replace("in its row below", "in [docs/traceability.md](docs/traceability.md)"), "", END]
    readme = (ROOT / "README.md").read_text()
    head, rest = readme.split(BEGIN, 1)
    (ROOT / "README.md").write_text(head + "\n".join(out) + rest.split(END, 1)[1])


if __name__ == "__main__":
    update([json.loads(p.read_text()) for p in sorted((ROOT / "results").glob("*.json"))])
