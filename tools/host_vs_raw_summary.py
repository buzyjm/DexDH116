"""Teleop-style (host, rate-limited) results vs the raw large-step results.

Central question: do the early/false completions seen with one large raw step
still occur when the same motion is delivered as incremental frame-by-frame
targets, the way teleop sends them?
"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
import numpy as np

import argparse
_ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0] if __doc__ else None)
_ap.add_argument("--data", default="data/benchmarks", help="directory holding the *.traces.json / *.csv outputs")
_args = _ap.parse_args()
ROOT = Path(_args.data)
HOST = ["h_im150", "h_im200", "h_rp150", "h_rp200", "h_tabd150", "h_tflex150"]
RAW = ["sr_fingers", "sr_thumb_abd", "sr_thumb_flex", "sr_index3"]

def load(stems, tag):
    out = []
    for s in stems:
        p = ROOT / f"{s}.traces.json"
        if not p.exists():
            print(f"  MISSING {p.name}")
            continue
        for x in json.load(open(p)):
            r = dict(x["result"])
            r["_src"], r["_mode"] = s, tag
            r["_moved"] = abs(max(x["angle"]) - min(x["angle"])) > 0.5
            r["_trace"] = x
            out.append(r)
    return out

print("loading host runs:");  host = load(HOST, "host")
print("loading raw runs:");   raw = load(RAW, "raw")
print(f"{len(host)} host steps, {len(raw)} raw steps")
if not host:
    sys.exit("no host results yet")

fin = lambda v: v is not None and v == v
med = lambda xs: float(np.median([x for x in xs if fin(x)])) if any(fin(x) for x in xs) else float("nan")
trav = lambda r: abs(r["target_deg"] - r["start_deg"])

# ------------------------------------------------------------ headline
nf = sum(bool(r["false_completion"]) for r in host)
nt = sum(1 for r in host if fin(r.get("early_reached_ms")) and not r["false_completion"])
nr = sum(1 for r in host if not r["reached"])
print(f"\n=== HEADLINE: host/incremental steps: {len(host)-nr}/{len(host)} reached, "
      f"{nf} terminal false completions, {nt} transient early-reached ===")

# ------------------------------------------------- per joint x velocity x kick
print("\n=== HOST RESULTS: joint x firmware velocity x stall kick ===")
hdr = (f"{'joint':<16}{'v':>4}{'kick':>5}{'n':>4}{'reached':>9}{'falseC':>7}"
       f"{'sust_cl':>8}{'sust_op':>8}{'|err|med':>9}{'|err|max':>9}"
       f"{'lagmax':>7}{'lagfin':>7}{'cmd_done':>9}{'stop':>7}{'cur':>6}")
print(hdr); print("-" * len(hdr))
key = lambda r: (r["joint"], r["velocity_setting"], bool(r["stall_kick"]))
groups = {}
for r in host:
    groups.setdefault(key(r), []).append(r)
for k in sorted(groups):
    rs = groups[k]
    cl = [r for r in rs if r["direction"] == "close"]
    op = [r for r in rs if r["direction"] == "open"]
    errs = [abs(r["final_err_deg"]) for r in rs]
    print(f"{rs[0]['joint_name']:<16}{k[1]:>4.0f}{('Y' if k[2] else 'n'):>5}{len(rs):>4}"
          f"{sum(bool(r['reached']) for r in rs):>5}/{len(rs):<3}"
          f"{sum(bool(r['false_completion']) for r in rs):>7}"
          f"{med([r['sustained_vel_deg_s'] for r in cl]):>8.0f}"
          f"{med([r['sustained_vel_deg_s'] for r in op]):>8.0f}"
          f"{med(errs):>9.2f}{max(errs):>9.2f}"
          f"{med([r['cmd_lag_max'] for r in rs]):>7.1f}"
          f"{med([r['cmd_lag_final'] for r in rs]):>7.2f}"
          f"{med([r['cmd_done_ms'] for r in rs]):>9.0f}"
          f"{med([r['stop_ms'] for r in rs]):>7.0f}"
          f"{med([r['peak_current'] for r in rs]):>6.0f}")

# ------------------------------------------------------- raw vs host, matched
print("\n=== RAW (one big step) vs HOST (incremental) for the same motion ===")
print(f"{'joint':<16}{'v':>4}{'dir':>6}{'step':>6}{'raw':>12}{'host kick=n':>14}{'host kick=Y':>14}")
def outcome(rs):
    if not rs:
        return "-"
    ok = sum(bool(r["reached"]) for r in rs)
    if ok == len(rs):
        return f"ok {ok}/{len(rs)}"
    worst = max(abs(r["final_err_deg"]) for r in rs if not r["reached"])
    return f"FAIL {ok}/{len(rs)} ({worst:.0f}d)"
combos = sorted({(r["joint"], r["velocity_setting"], r["direction"], round(trav(r) / 10) * 10)
                 for r in host})
for j, v, d, st in combos:
    sel = lambda pool, kick=None: [r for r in pool
                                   if r["joint"] == j and r["velocity_setting"] == v
                                   and r["direction"] == d and round(trav(r) / 10) * 10 == st
                                   and (kick is None or bool(r["stall_kick"]) == kick)]
    rw, hn, hy = sel(raw), sel(host, False), sel(host, True)
    name = (hn or hy)[0]["joint_name"]
    print(f"{name:<16}{v:>4.0f}{d:>6}{st:>6.0f}{outcome(rw):>12}{outcome(hn):>14}{outcome(hy):>14}")

# --------------------------------------------------------- stall kick effect
print("\n=== STALL KICK on/off (host only) ===")
print(f"{'joint':<16}{'v':>4}{'reach n':>9}{'reach Y':>9}{'|err|med n':>11}{'|err|med Y':>11}"
      f"{'|err|max n':>11}{'|err|max Y':>11}{'cur n':>7}{'cur Y':>7}")
for (j, v) in sorted({(r["joint"], r["velocity_setting"]) for r in host}):
    a = [r for r in host if r["joint"] == j and r["velocity_setting"] == v and not r["stall_kick"]]
    b = [r for r in host if r["joint"] == j and r["velocity_setting"] == v and r["stall_kick"]]
    if not a or not b:
        continue
    ea = [abs(r["final_err_deg"]) for r in a]; eb = [abs(r["final_err_deg"]) for r in b]
    print(f"{a[0]['joint_name']:<16}{v:>4.0f}{sum(bool(r['reached']) for r in a):>5}/{len(a):<3}"
          f"{sum(bool(r['reached']) for r in b):>5}/{len(b):<3}"
          f"{med(ea):>11.2f}{med(eb):>11.2f}{max(ea):>11.2f}{max(eb):>11.2f}"
          f"{med([r['peak_current'] for r in a]):>7.0f}{med([r['peak_current'] for r in b]):>7.0f}")

# ------------------------------------------------------------- any failures
fails = [r for r in host if not r["reached"]]
print(f"\n=== HOST STEPS THAT DID NOT REACH ({len(fails)}) ===")
for r in fails:
    print(f"  {r['joint_name']} {r['direction']} {r['step_deg']:+.0f} @v{r['velocity_setting']:.0f} "
          f"kick={'Y' if r['stall_kick'] else 'n'} [{r['_src']}]")
    print(f"     final {r['final_deg']:.2f} err {r['final_err_deg']:+.2f} | stop {r['stop_ms']:.0f} ms "
          f"| cmd_done {r['cmd_done_ms']:.0f} ms | lag max {r['cmd_lag_max']:.1f} final {r['cmd_lag_final']:.2f}")
    print(f"     target_after {r['target_after']} reached_after {r['reached_after']} "
          f"status [{r['statuses_seen']}]->{r['status_after']} | cur {r['peak_current']:.0f}/{r['max_current']} "
          f"| alarms [{r['alarm_after']}] during {r['alarm_during']}")
    print(f"     => {r.get('stop_class')}")
    t = r["_trace"]
    print("     samples around the stop:")
    for i in range(len(t["t_ms"])):
        if t["t_ms"][i] < r["stop_ms"] - 150 or t["t_ms"][i] > r["stop_ms"] + 250:
            continue
        print(f"       t={t['t_ms'][i]:7.1f} angle {t['angle'][i]:6.2f} cmd {t['cmd'][i]:6.2f} "
              f"tgt {t['target'][i]:5.1f} rch {t['reached'][i]} st {t['status'][i]} "
              f"cur {t['current'][i]:4.0f}")

# --------------------------------------------------- command-following lag
print("\n=== COMMAND-FOLLOWING LAG (how far the ramped target outran the joint) ===")
print(f"{'joint':<16}{'v':>4}{'dir':>6}{'lag_max':>9}{'lag_med':>9}{'lag_final':>10}{'reached':>9}")
for k in sorted({(r["joint"], r["velocity_setting"], r["direction"]) for r in host}):
    rs = [r for r in host if (r["joint"], r["velocity_setting"], r["direction"]) == k]
    print(f"{rs[0]['joint_name']:<16}{k[1]:>4.0f}{k[2]:>6}"
          f"{med([r['cmd_lag_max'] for r in rs]):>9.1f}"
          f"{med([r['cmd_lag_med'] for r in rs]):>9.1f}"
          f"{med([r['cmd_lag_final'] for r in rs]):>10.2f}"
          f"{sum(bool(r['reached']) for r in rs):>5}/{len(rs):<3}")

alarmed = [r for r in host if r["alarm_during"] or any(c != "0" for c in r["alarm_after"].split(";"))]
print(f"\n=== HOST STEPS WITH ANY ALARM ({len(alarmed)}) ===")
for r in alarmed:
    print(f"  {r['joint_name']} {r['direction']} @v{r['velocity_setting']:.0f}: during {r['alarm_during']}, "
          f"after [{r['alarm_after']}]")

print("\n=== VERDICT INPUT: index/middle at 200 under incremental control ===")
for j in (2, 3):
    rs = [r for r in host if r["joint"] == j and r["velocity_setting"] == 200.0]
    if not rs:
        print(f"  joint {j}: no 200 data")
        continue
    errs = [abs(r["final_err_deg"]) for r in rs]
    print(f"  {rs[0]['joint_name']}: {sum(bool(r['reached']) for r in rs)}/{len(rs)} reached, "
          f"{sum(bool(r['false_completion']) for r in rs)} false completions, "
          f"|err| max {max(errs):.2f} deg, peak current max {max(r['peak_current'] for r in rs):.0f}")
