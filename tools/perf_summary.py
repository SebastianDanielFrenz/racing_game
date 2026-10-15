#!/usr/bin/env python3
"""Summarises the frame-time lines of a racing_game log (RG_FRAME_STATS, main.gd).

Usage:  python tools/perf_summary.py godot.log [--skip-s 20]

Prints the median of every RG_FRAME_STATS field over the run (fps, frame ms, draw calls, primitives and the per-phase
probes: scene_span, ph_post, ph_draw, render_cpu, render_gpu, ...). For a run with RG_PERF_AB=<features|all> it
prints one row per feature: frame ms and the main phases with the feature on and off, and the saving. The first
line after every switch is dropped (it straddles the switch). --skip-s drops the first seconds after the world
starts (loading, shader compiles).
"""
import re
import statistics
import sys

STATS = re.compile(r"RG_FRAME_STATS (.*)")
PROBES = re.compile(r"probes_avg/max_ms=\[(.*)\]")
AB = re.compile(r" ab=([a-z0-9_]+):(on|off)")
KEYS = ["frame_ms_avg", "fps", "draw_calls", "objects", "primitives"]
PHASES = ["scene_span", "ph_post", "ph_draw", "render_cpu", "render_gpu", "ph_loop", "outside"]


def parse(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = STATS.search(line)
            if not m:
                continue
            text = m.group(1)
            row = {}
            for k in KEYS:
                km = re.search(r"\b%s=([0-9.]+)" % k, text)
                if km:
                    row[k] = float(km.group(1))
            pm = PROBES.search(text)
            if pm:
                for part in pm.group(1).split(", "):
                    if "=" in part:
                        name, val = part.split("=", 1)
                        row[name] = float(val.split("/")[0])
            rm = re.search(r"\brenderer=(\S+)", text)
            row["renderer"] = rm.group(1) if rm else "?"
            am = AB.search(text)
            row["ab"] = (am.group(1), am.group(2)) if am else None
            rows.append(row)
    return rows


def med(rows, key):
    # A probe that did not fire in a second is absent from that line: it cost 0 ms there.
    vals = [r.get(key, 0.0) for r in rows] if key not in KEYS else [r[key] for r in rows if key in r]
    return statistics.median(vals) if vals else float("nan")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    skip = 20
    if "--skip-s" in sys.argv:
        skip = int(sys.argv[sys.argv.index("--skip-s") + 1])
    rows = parse(sys.argv[1])[skip:]
    if not rows:
        print("no RG_FRAME_STATS lines (after skipping %d)" % skip)
        return 1
    plain = [r for r in rows if r["ab"] is None or r["ab"][1] == "on"]
    print("lines: %d (first %d skipped), renderer %s" % (len(rows), skip, ", ".join(sorted({r["renderer"] for r in rows}))))
    print("median over the run (feature on):")
    for k in KEYS + PHASES:
        print("  %-14s %10.2f" % (k, med(plain, k)))
    probes = sorted({k for r in plain for k in r if k not in KEYS + PHASES + ["ab", "renderer"]}, key=lambda k: -med(plain, k))
    print("scripts and draw probes (median ms):")
    for k in probes:
        v = med(plain, k)
        if statistics.fmean(r.get(k, 0.0) for r in plain) >= 0.02:
            print("  %-18s %6.2f   (mean %.3f)" % (k, v, statistics.fmean(r.get(k, 0.0) for r in plain)))
    # RG_PERF_AB: drop the first line after each switch.
    groups = {}
    prev = None
    for r in rows:
        if r["ab"] is None:
            continue
        if r["ab"] != prev:
            prev = r["ab"]
            continue
        groups.setdefault(r["ab"][0], {"on": [], "off": []})[r["ab"][1]].append(r)
    if groups:
        print("\nRG_PERF_AB (median of each state; saving = on - off):")
        cols = ["frame_ms_avg"] + PHASES[:5] + ["draw_calls", "primitives"]
        print("  %-16s %s" % ("feature", " ".join("%18s" % c for c in cols)))
        for feat, g in groups.items():
            if not g["on"] or not g["off"]:
                continue
            cells = []
            for c in cols:
                on, off = med(g["on"], c), med(g["off"], c)
                if c == "primitives":
                    cells.append("%8.2fM->%6.2fM" % (on / 1e6, off / 1e6))
                else:
                    cells.append("%8.2f->%7.2f" % (on, off))
            print("  %-16s %s" % (feat, " ".join("%18s" % c for c in cells)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
