"""Per-joint 150 vs 200 comparison over the usable-speed sweep traces."""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
import numpy as np

import argparse
_ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0] if __doc__ else None)
_ap.add_argument("--data", default="data/benchmarks", help="directory holding the *.traces.json / *.csv outputs")
_args = _ap.parse_args()
ROOT = Path(_args.data)
FILES = ["sr_fingers.traces.json", "sr_thumb_abd.traces.json", "sr_thumb_flex.traces.json"]

rows = []
for f in FILES:
    p = ROOT / f
    if not p.exists():
        print(f"MISSING {f}")
        continue
    for x in json.load(open(p)):
        r = dict(x["result"])
        r["_file"] = f
        rows.append(r)
print(f"{len(rows)} steps loaded")

def fin(v):
    return v is not None and v == v

def med(vals):
    vals = [v for v in vals if fin(v)]
    return float(np.median(vals)) if vals else float("nan")

def travel(r):
    return abs(r["target_deg"] - r["start_deg"])

# ---------------------------------------------------------------- resolution
p50 = [r["update_p50_ms"] for r in rows if fin(r["update_p50_ms"])]
p95 = [r["update_p95_ms"] for r in rows if fin(r["update_p95_ms"])]
print(f"\nfeedback update interval across all steps: p50 median {med(p50):.1f} ms, "
      f"worst step p95 {max(p95):.1f} ms")

joints = sorted({r["joint"] for r in rows})
names = {r["joint"]: r["joint_name"] for r in rows}

# ------------------------------------------------------------ per-joint table
print("\n=== PER JOINT, 150 vs 200 ===")
hdr = (f"{'joint':<16}{'v':>4}{'n':>4}{'reached':>9}{'falseC':>7}{'trans':>6}"
       f"{'sust_cl':>8}{'sust_op':>8}{'peak':>6}{'|err|med':>9}{'|err|max':>9}"
       f"{'cur_cl':>7}{'cur_op':>7}{'curmax':>7}{'t90cl':>7}{'t90op':>7}")
print(hdr); print("-" * len(hdr))
summary = {}
for j in joints:
    for v in (150.0, 200.0):
        rs = [r for r in rows if r["joint"] == j and r["velocity_setting"] == v]
        if not rs:
            continue
        cl = [r for r in rs if r["direction"] == "close"]
        op = [r for r in rs if r["direction"] == "open"]
        errs = [abs(r["final_err_deg"]) for r in rs if fin(r["final_err_deg"])]
        falsec = sum(bool(r.get("false_completion")) for r in rs)
        trans = sum(1 for r in rs if fin(r.get("early_reached_ms")) and not r.get("false_completion"))
        rec = dict(
            n=len(rs), reached=sum(bool(r["reached"]) for r in rs), falsec=falsec, trans=trans,
            sust_cl=med([r["sustained_vel_deg_s"] for r in cl]),
            sust_op=med([r["sustained_vel_deg_s"] for r in op]),
            peak=med([r["peak_vel_deg_s"] for r in rs]),
            err_med=med(errs), err_max=max(errs) if errs else float("nan"),
            cur_cl=med([r["peak_current"] for r in cl]), cur_op=med([r["peak_current"] for r in op]),
            cur_max=max(r["peak_current"] for r in rs),
            t90_cl=med([r["t90_ms"] for r in cl]), t90_op=med([r["t90_ms"] for r in op]),
        )
        summary[(j, v)] = rec
        print(f"{names[j]:<16}{v:>4.0f}{rec['n']:>4}{rec['reached']:>5}/{rec['n']:<3}{falsec:>7}{trans:>6}"
              f"{rec['sust_cl']:>8.0f}{rec['sust_op']:>8.0f}{rec['peak']:>6.0f}{rec['err_med']:>9.2f}"
              f"{rec['err_max']:>9.2f}{rec['cur_cl']:>7.0f}{rec['cur_op']:>7.0f}{rec['cur_max']:>7.0f}"
              f"{rec['t90_cl']:>7.0f}{rec['t90_op']:>7.0f}")

# --------------------------------------------------- per joint, per step size
print("\n=== PER JOINT x STEP: t90 and sustained, 150 -> 200 (t90 is +-64 ms) ===")
print(f"{'joint':<16}{'step':>5}{'dir':>6}{'t90_150':>9}{'t90_200':>9}{'dt90':>7}"
      f"{'sust150':>9}{'sust200':>9}{'ok150':>7}{'ok200':>7}")
for j in joints:
    steps = sorted({round(travel(r) / 5) * 5 for r in rows if r["joint"] == j})
    for s in steps:
        for d in ("close", "open"):
            def sel(v):
                return [r for r in rows if r["joint"] == j and r["velocity_setting"] == v
                        and r["direction"] == d and round(travel(r) / 5) * 5 == s]
            a, b = sel(150.0), sel(200.0)
            if not a and not b:
                continue
            ta, tb = med([r["t90_ms"] for r in a]), med([r["t90_ms"] for r in b])
            print(f"{names[j]:<16}{s:>5.0f}{d:>6}{ta:>9.0f}{tb:>9.0f}{tb - ta:>7.0f}"
                  f"{med([r['sustained_vel_deg_s'] for r in a]):>9.0f}"
                  f"{med([r['sustained_vel_deg_s'] for r in b]):>9.0f}"
                  f"{sum(bool(r['reached']) for r in a):>4}/{len(a):<2}"
                  f"{sum(bool(r['reached']) for r in b):>4}/{len(b):<2}")

# ------------------------------------------------------------------- failures
fails = [r for r in rows if not r["reached"]]
print(f"\n=== UNREACHED STEPS ({len(fails)}) ===")
for r in fails:
    print(f"  {r['joint_name']} {r['direction']} {r['step_deg']:+.0f} @ {r['velocity_setting']:.0f}: "
          f"final {r['final_deg']:.2f} err {r['final_err_deg']:+.2f} | target_after {r['target_after']} "
          f"reached_after {r['reached_after']} status [{r['statuses_seen']}] -> {r['status_after']} | "
          f"current {r['peak_current']:.0f}/{r['max_current']} | alarms [{r['alarm_after']}] "
          f"during {r['alarm_during']}")
    print(f"     label: {r.get('stop_class')}")

trans = [r for r in rows if fin(r.get("early_reached_ms")) and not r.get("false_completion")]
print(f"\n=== TRANSIENT EARLY-REACHED EVENTS ({len(trans)}) ===")
for r in trans:
    print(f"  {r['joint_name']} {r['direction']} {r['step_deg']:+.0f} @ {r['velocity_setting']:.0f}: "
          f"reached=1 at {r['early_reached_ms']:.0f} ms, {r['early_reached_err_deg']:.1f} deg out; "
          f"final err {r['final_err_deg']:+.2f}")

alarmed = [r for r in rows if r["alarm_during"] or any(c != "0" for c in r["alarm_after"].split(";"))]
print(f"\n=== STEPS WITH ANY ALARM ({len(alarmed)}) ===")
for r in alarmed:
    print(f"  {r['joint_name']} {r['direction']} {r['step_deg']:+.0f} @ {r['velocity_setting']:.0f}: "
          f"during {r['alarm_during']} at {r['alarm_during_ms']} ms, after [{r['alarm_after']}]")

skipped = []
for f in ("sweep_fingers.log", "sweep_thumb_abd.log", "sweep_thumb_flex.log"):
    p = ROOT / f
    if not p.exists():
        p = ROOT.parent / "logs" / f          # data/logs beside data/benchmarks
    if p.exists():
        skipped += [l.strip() for l in p.read_text().splitlines() if "skipping" in l]
print(f"\n=== ESCALATION SKIPS ({len(skipped)}) ===")
for l in skipped:
    print("  " + l)

# ------------------------------------------------------------------- verdict
print("\n=== 200 vs 150 DELTAS PER JOINT ===")
print(f"{'joint':<16}{'sust_cl %':>10}{'sust_op %':>10}{'reach150':>9}{'reach200':>9}"
      f"{'falseC200':>10}{'curmax150':>10}{'curmax200':>10}{'errmax200':>10}")
for j in joints:
    a, b = summary.get((j, 150.0)), summary.get((j, 200.0))
    if not a or not b:
        continue
    pc = lambda x, y: 100 * (y - x) / x if fin(x) and fin(y) and x else float("nan")
    print(f"{names[j]:<16}{pc(a['sust_cl'], b['sust_cl']):>10.0f}{pc(a['sust_op'], b['sust_op']):>10.0f}"
          f"{a['reached']:>5}/{a['n']:<3}{b['reached']:>5}/{b['n']:<3}{b['falsec']:>10}"
          f"{a['cur_max']:>10.0f}{b['cur_max']:>10.0f}{b['err_max']:>10.2f}")
