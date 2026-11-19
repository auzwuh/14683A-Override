"""Collect the numbers for the final report from sim/out/final.
usage: python final_analysis.py [section ...]   sections: headline steps odom resets timing lift
Reads summary_*.kv / events_*.json / log_*.txt / pose_*.csv written by robot_sim. Prints plain text."""
import csv, json, os, re, sys, collections, statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out", "final")
ROUTE = os.path.join(HERE, "..", "..", "src", "skills_dr4b.cpp")
N = 30

# ---------------------------------------------------------------- route structure
src = open(ROUTE).read().split("\n")
REL, INT, RST, MOT, DELAY = [], [], [], {}, {}
for i, s in enumerate(src, 1):
    if "rclaw::release()" in s: REL.append(i)
    if "rclaw::intake()" in s: INT.append(i)
    if "applyImmediateCorrectionAuto" in s or "resetNow" in s: RST.append(i)
    m = re.search(r"chassis\.(moveToPoint|turnToPoint)\(\s*([-0-9.]+)f?,\s*([-0-9.]+)f?.*?\.timeout = (\d+)", s)
    if m: MOT[i] = (m[1], float(m[2]), float(m[3]), int(m[4]))
    m = re.search(r"pros::delay\((\d+)\)", s)
    if m: DELAY[i] = int(m[1])
PLACE = ["P1 preload Pin -> Short G1 [0 on Goal]", "P2 top Loader 1 (Cup+Pin) -> Short G1 [1]", "P3 top Loader 2 (Cup+Pin) -> Short G1 [3]",
         "P4 top Loader 3 (Cup+Pin) -> Short G1 [5]", "P5 top Loader 4 (Pin) -> Alliance G2 [0]", "P6 floor D48 (Cup+Pin) -> Alliance G2 [1]",
         "P7 floor D24 (Cup+Pin) -> Alliance G2 [3]", "P8 bottom Loader 1 (Cup+Pin) -> Alliance G2 [5]", "P9 bottom Loader 2 (Cup+Pin) -> Alliance G2 [7]",
         "P10 bottom Loader 3 (Pin) -> Tall [0]", "P11 floor D96 (Cup+Pin) -> Tall [1]", "P12 floor D120 (Cup+Pin) -> Tall [3]"]
GRAB = ["G1 top Loader load 1", "G2 top Loader load 2", "G3 top Loader load 3", "G4 top Loader load 4 (Pin)", "G5 floor D48",
        "G6 floor D24", "G7 bottom Loader load 1", "G8 bottom Loader load 2", "G9 bottom Loader load 3 (Pin)", "G10 floor D96", "G11 floor D120"]
RESET = ["R1 top Loader", "R2 Short G1 lane", "R3 top Loader", "R4 Short G1 lane", "R5 top Loader", "R6 Short G1 lane", "R7 top Loader",
         "R8 Alliance G2 lane (x -12)", "R9 after back square (-46.8, 62)", "R10 Alliance G2 lane (x -34.7)", "R11 bottom Loader",
         "R12 Alliance G2 lane", "R13 bottom Loader", "R14 Alliance G2 lane", "R15 bottom Loader"]
assert len(REL) == 12 and len(INT) == 11 and len(RST) == 15, (len(REL), len(INT), len(RST))


def first_ge(lines, ln):
    for k, v in enumerate(lines):
        if v >= ln: return k
    return None


# ---------------------------------------------------------------- loading
def kv(tag):
    d = {}
    p = os.path.join(OUT, f"summary_{tag}.kv")
    if not os.path.exists(p): return None
    for line in open(p):
        a = line.split()
        if a: d[a[0]] = a[1:]
    return d


def events(tag): return json.load(open(os.path.join(OUT, f"events_{tag}.json")))
def summary(tag): return json.load(open(os.path.join(OUT, f"summary_{tag}.json")))


def log(tag):
    out = []
    for line in open(os.path.join(OUT, f"log_{tag}.txt"), errors="replace"):
        m = re.match(r"\[\s*([0-9.]+) (\S+)\s*\] (.*)", line.rstrip("\n"))
        if m: out.append((float(m[1]), m[2], m[3]))
    return out


def pose(tag):
    return list(csv.DictReader(open(os.path.join(OUT, f"pose_{tag}.csv"))))


def norm(why):
    return re.sub(r"\(.*?\)", "", re.sub(r"[-+]?\d+\.\d+", "#", why)).strip()


def stats(v, f="%.1f"):
    if not v: return "n=0"
    v = sorted(v)
    return ("mean " + f + " median " + f + " min " + f + " max " + f) % (sum(v) / len(v), v[len(v) // 2], v[0], v[-1]) + f" (n={len(v)})"


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))]


class Run:
    def __init__(self, tag):
        self.tag = tag
        self.ev = events(tag)
        self.kv = kv(tag)
        self.t0 = next(e["t"] for e in self.ev if e["type"] == "auton_start")
        last = 0
        self.grabs = {}      # grab index -> dict(ok, e, tips)
        self.places = {}     # placement index -> event
        self.motions = [e for e in self.ev if e["type"] == "motion"]
        pend_tip = []
        for e in self.ev:
            t = e["type"]
            if t == "motion": last = e["line"]
            elif t == "tipped": pend_tip.append(e)
            elif t in ("grab", "grab_miss"):
                # the intake that this event answers: the last rclaw::intake() at or before the motion that just ended
                # (+8 lines: a floor creep is started 3 lines before its intake, a Loader approach 2 lines before)
                c = [i for i, ln in enumerate(INT) if ln <= last + 8]
                if not c: continue
                k = c[-1]
                if k in self.grabs and self.grabs[k]["ok"]: continue
                self.grabs[k] = {"ok": t == "grab", "e": e, "tips": pend_tip, "t": e["t"] - self.t0}
                pend_tip = []
            elif t == "placement":
                self.places[e["n"] - 1] = e
        self.last_line = last


def runs(batch):
    return [Run(f"mc_{batch}_seed{s}") for s in range(1, N + 1)]


BATCH = {"code": "main.cpp gains", "tuned": "--tuned-drive", "tuned_nocont": "--tuned-drive --no-cont-track", "code_nocont": "main.cpp gains --no-cont-track"}


# ---------------------------------------------------------------- 1 headline
def headline():
    for b, lab in BATCH.items():
        R = runs(b)
        g = lambda k: [float(r.kv[k][0]) for r in R]
        pts = g("points"); pins = g("pins")
        print(f"\n== {b} [{lab}] 30 seeds")
        print("points:", stats(pts), "sd %.1f" % st.pstdev(pts), "| distribution", dict(sorted(collections.Counter(int(p) for p in pts).items())))
        print("points_if_owned (Toggle ignored):", stats(g("points_if_owned")))
        print("Pins seated distribution:", dict(sorted(collections.Counter(int(p) for p in pins).items())), "mean %.2f" % (sum(pins) / N))
        print("Midfield touching at rest: %d/30, at the 60 s cut: %d/30" % (sum(g("midfield")), sum(g("midfield_at_cut"))))
        print("route completed inside 60 s: %d/30; route_end_s" % sum(g("completed")), stats(g("route_end_s")))
        print("clock check A reached: %d/30, B reached: %d/30 (values %s / %s; -1 = never reached)" % (
            sum(1 for v in g("check_0") if v >= 0), sum(1 for v in g("check_1") if v >= 0), sorted(set(g("check_0"))), sorted(set(g("check_1")))))
        print("releases issued before the cut:", dict(sorted(collections.Counter(int(p) for p in g("placements")).items())))
        print("last route source line whose motion ended before the cut:", stats([r.last_line for r in R], "%.0f"))
        print("Toggle sweeps:", dict(sorted(collections.Counter(int(p) for p in g("toggle_sweeps")).items())))
        print("heading lost (>45 deg at end): %d/30; final heading err" % sum(g("lost_heading_over_45")), stats(g("head_err_final")))
        print("speed at the cut in/s:", stats(g("speed_at_cut")))
        print("motions timed out:", stats(g("motions_timed_out"), "%.0f"), "| motions started", stats([len(r.motions) for r in R], "%.0f"))
        b2 = b + "_120s"
        if os.path.exists(os.path.join(OUT, f"summary_mc_{b2}_seed1.kv")):
            K = [kv(f"mc_{b2}_seed{s}") for s in range(1, N + 1)]
            S = [summary(f"mc_{b2}_seed{s}") for s in range(1, N + 1)]
            print("  INFO run with the 60 s cut moved to 120 s:")
            for i, nm in ((0, "A (kSkipALegAfterMs 54.0 s)"), (1, "B (kLastLegStartMs 53.5 s)")):
                res = collections.Counter(s["clock_checks"][i]["result"] for s in S)
                el = [s["clock_checks"][i]["elapsed_s"] for s in S if s["clock_checks"][i]["elapsed_s"] >= 0]
                print(f"    check {nm}: {dict(res)}; clock at the check s: {stats(el)}")
            print("    route end s:", stats([float(k["route_end_s"][0]) for k in K]), "| completed", sum(float(k["completed"][0]) for k in K))
            print("    points:", stats([float(k["points"][0]) for k in K]), "| midfield", sum(float(k["midfield"][0]) for k in K))


# ---------------------------------------------------------------- 2 placements and grabs
def steps():
    for b, lab in BATCH.items():
        if b == "code_nocont": continue
        R = runs(b)
        print(f"\n== {b} [{lab}] placements (release reached before the cut / claw not empty / seated)")
        for k, name in enumerate(PLACE):
            ev = [r.places[k] for r in R if k in r.places]
            held = [e for e in ev if e["object"] != "nothing held"]
            ok = [e for e in held if e["ok"]]
            why = collections.Counter(norm(e["why"]) for e in ev if not e["ok"])
            rad = [e["radial_err_in"] for e in held]
            wrong = collections.Counter(e["goal"] for e in held)
            print(f"  {name}: reached {len(ev)}/30, holding {len(held)}, seated {len(ok)} ({100*len(ok)/30:.0f}% of runs)"
                  + (f" | radial {stats(rad, '%.2f')}" if rad else "") + (f" | fail: {why.most_common(3)}" if why else "")
                  + (f" | nearest Goal {dict(wrong)}" if held else ""))
        print(f"== {b} [{lab}] grabs")
        for k, name in enumerate(GRAB):
            g = [r.grabs[k] for r in R if k in r.grabs]
            ok = [x for x in g if x["ok"]]
            reasons = collections.Counter()
            for x in g:
                if x["ok"]: continue
                d = x["e"]["detail"]
                m = re.search(r"along ([-+0-9.]+) lat ([-+0-9.]+)", d)
                tb = [t["by"] for t in x["tips"]]
                if tb: reasons["tipped by " + tb[-1]] += 1
                elif not m: reasons["nothing near"] += 1
                else:
                    al, la = float(m[1]), float(m[2])
                    if abs(al) > 6 or abs(la) > 6: reasons["robot far from it (>6 in)"] += 1
                    elif abs(la) > 0.77: reasons["beside the throat (|lat| > 0.77)"] += 1
                    elif al > 1.75: reasons["stopped short (along > 1.75)"] += 1
                    else: reasons["other: " + d[:60]] += 1
            lat = [abs(x["e"]["lat_in"]) for x in ok]
            print(f"  {name}: attempted {len(g)}/30, grabbed {len(ok)} ({100*len(ok)/30:.0f}% of runs)" + (f" | |lat| at grab {stats(lat, '%.2f')}" if lat else "")
                  + (f" | miss: {reasons.most_common(4)}" if reasons else ""))


# ---------------------------------------------------------------- 3 odometry
def odom():
    import math
    for b, lab in BATCH.items():
        if b == "code_nocont": continue
        series = []
        for s in range(1, N + 1):
            rows = [r for r in pose(f"mc_{b}_seed{s}") if r["phase"] == "auton"]
            t0 = float(rows[0]["t"])
            series.append([(float(r["t"]) - t0, math.hypot(float(r["x"]) - float(r["odom_x"]), float(r["y"]) - float(r["odom_y"])),
                            abs((float(r["theta"]) - float(r["odom_theta"]) + 180) % 360 - 180)) for r in rows])
        print(f"\n== {b} [{lab}] odometry xy error (in) and heading error (deg) over time, 30 seeds: median / 90th pct / worst")
        for lo in range(0, 60, 5):
            xy = [max((e for t, e, h in sr if lo <= t < lo + 5), default=float("nan")) for sr in series]
            hd = [max((h for t, e, h in sr if lo <= t < lo + 5), default=float("nan")) for sr in series]
            print(f"  {lo:2d}-{lo+5:2d} s: xy {pct(xy,.5):6.2f} / {pct(xy,.9):6.2f} / {max(xy):6.2f}   heading {pct(hd,.5):6.2f} / {pct(hd,.9):6.2f} / {max(hd):6.2f}")
        mx = [max(e for t, e, h in sr) for sr in series]
        order = sorted(range(N), key=lambda i: mx[i])
        print("  per-seed max xy error: median seed %d (%.2f in), worst seed %d (%.2f in), best seed %d (%.2f in)" % (
            order[N // 2] + 1, mx[order[N // 2]], order[-1] + 1, mx[order[-1]], order[0] + 1, mx[order[0]]))
        # corrections
        R = runs(b)
        by = collections.defaultdict(list)
        for r in R:
            last = 0
            for e in r.ev:
                if e["type"] == "motion": last = e["line"]
                elif e["type"] == "set_pose" and e["source"] != "route":
                    key = e["source"]
                    if key == "stop_reset":
                        k = first_ge(RST, last)
                        key = RESET[k] if k is not None else "?"
                    elif key == "square":
                        key = "square FRONT at Toggle (line 111)" if last < 140 else "square BACK at (-46.8, 62) (line 163)"
                    by[key].append((e["xy_err_before"], e["xy_err_after"]))
        print("  pose writes: xy error before -> after (medians), and how many made it worse")
        for k in sorted(by, key=lambda s: (not s.startswith("R"), int(re.sub(r"\D", "", s.split()[0]) or 0), s)):
            v = by[k]
            bf = [a for a, _ in v]; af = [c for _, c in v]
            worse = sum(1 for a, c in v if c > a + 0.1)
            print(f"    {k}: n={len(v)} (in {len(v)}/30 runs max 1 each) before {pct(bf,.5):.2f} (max {max(bf):.2f}) -> after {pct(af,.5):.2f} (max {max(af):.2f}); worse by >0.1 in: {worse}")
        start = [e for r in R for e in r.ev if e["type"] == "set_pose" and e["source"] == "route"]
        print("  start pose error after chassis.setPose (in):", stats([e["xy_err_after"] for e in start], "%.2f"))


def resets():
    for b, lab in BATCH.items():
        if b == "code_nocont": continue
        table = collections.defaultdict(collections.Counter)
        applied = collections.Counter(); calls = collections.Counter()
        rej = []
        for s in range(1, N + 1):
            tag = f"mc_{b}_seed{s}"
            ev = events(tag)
            mot = [(e["t"], e["line"]) for e in ev if e["type"] == "motion"]
            def line_at(t):
                ln = 0
                for mt, ml in mot:
                    if mt <= t + 1e-6: ln = ml
                    else: break
                return ln
            seen = set()
            for t, task, msg in log(tag):
                if not msg.startswith("[reset]"): continue
                # a reset prints 150 to 300 ms after the arriving motion ended
                k = first_ge(RST, line_at(t - 0.02))
                name = RESET[k] if k is not None else "?"
                m = re.match(r"\[reset\] mount (\d) read ([-0-9.naN]+) pred ([-0-9.]+) -> (.*)", msg)
                if m:
                    why = m[4]
                    table[name][("front", "left", "right")[int(m[1])] + ": " + why] += 1
                    if (name, s) not in seen: seen.add((name, s)); calls[name] += 1
                    if why.startswith("REJECT"): rej.append((s, round(t, 2), name, m[1], float(m[2]), float(m[3])))
                elif "applied" in msg: applied[name] += 1
                else:
                    table[name][msg] += 1
                    if (name, s) not in seen: seen.add((name, s)); calls[name] += 1
        print(f"\n== {b} [{lab}] stop resets per stop over 30 seeds (ran / wrote a correction), then per-mount outcomes")
        for name in RESET:
            if not calls[name]: print(f"  {name}: never reached"); continue
            ok = sum(c for k, c in table[name].items() if k.endswith(": ok"))
            rj = sum(c for k, c in table[name].items() if "REJECT" in k)
            print(f"  {name}: ran {calls[name]}/30, applied {applied[name]}, mount readings ok {ok}, REJECT {rj} | " +
                  "; ".join(f"{k} x{c}" for k, c in sorted(table[name].items(), key=lambda kc: -kc[1])[:5]))
        print(f"  REJECT lines: {len(rej)}")
        for r in rej[:12]: print("    seed %d t %.2f %s mount %s read %.2f pred %.2f (diff %+.2f)" % (*r, r[4] - r[5]))


# ---------------------------------------------------------------- 4 timing
PH = [("start -> release 1 (preload)", 0, REL[0]), ("-> release 2", REL[0], REL[1]), ("-> release 3", REL[1], REL[2]), ("-> release 4", REL[2], REL[3]),
      ("-> release 5 (Loader, Toggle, Alliance)", REL[3], REL[4]), ("-> release 6 (D48)", REL[4], REL[5]), ("-> release 7 (back square, D24)", REL[5], REL[6]),
      ("-> release 8", REL[6], REL[7]), ("-> release 9", REL[7], REL[8]), ("-> release 10 (Tall, clock check A)", REL[8], REL[9]),
      ("-> release 11 (leg A, D96, check B)", REL[9], REL[10]), ("-> release 12 (leg B, D120)", REL[10], REL[11])]


def atticus_est(lo, hi):
    """ATTICUS pads every timeout to 1.9 x its own duration estimate (config.json pad_factor), so timeout / 1.9 is its estimate."""
    mv = sum(v[3] for ln, v in MOT.items() if lo < ln <= hi) / 1.9 / 1000.0
    dl = sum(v for ln, v in DELAY.items() if lo < ln <= hi + 1) / 1000.0   # the 200 ms after a release belongs to the phase it ends
    return mv, dl


def timing():
    tot = atticus_est(0, 10**6)
    print("ATTICUS estimate rebuilt as sum(timeout)/1.9 + sum(pros::delay): moves %.2f s + delays %.2f s = %.2f s (file says 54.79 s)" % (tot[0], tot[1], sum(tot)))
    cum = 0
    est = []
    for name, lo, hi in PH:
        a = atticus_est(lo, hi); cum += sum(a); est.append((sum(a), cum))
    for b in ("code_120s", "tuned_120s", "tuned_nocont_120s", "code", "tuned", "tuned_nocont"):
        if not os.path.exists(os.path.join(OUT, f"summary_mc_{b}_seed1.kv")): continue
        R = runs(b)
        print(f"\n== {b}: clock (s since markStart) at each release, 30 seeds, vs ATTICUS")
        prev = None
        for k, (name, lo, hi) in enumerate(PH):
            ts = [r.places[k]["t"] - r.t0 for r in R if k in r.places]
            # release event fires ~80 ms after rclaw::release(); close enough for 0.1 s resolution
            if not ts: print(f"  {name}: never reached"); continue
            dur = [r.places[k]["t"] - (r.places[k - 1]["t"] if k else r.t0) for r in R if k in r.places and (k == 0 or k - 1 in r.places)]
            print(f"  {name}: ATTICUS phase {est[k][0]:.2f} s (cum {est[k][1]:.2f}) | sim phase median {pct(dur,.5):.2f} (min {min(dur):.2f} max {max(dur):.2f}) cum median {pct(ts,.5):.2f} max {max(ts):.2f} | n={len(ts)}")
        # where the time goes: per motion kind
        mk = collections.defaultdict(list)
        for r in R:
            tm = collections.Counter(); n = collections.Counter(); to = collections.Counter()
            for e in r.motions:
                tm[e["kind"]] += e["dur_ms"]; n[e["kind"]] += 1; to[e["kind"]] += e["timed_out"]
            for kd in tm: mk[kd].append((tm[kd] / 1000.0, n[kd], to[kd]))
        for kd, v in mk.items():
            print(f"  {kd}: time in motion per run median {pct([a for a,_,_ in v],.5):.1f} s over {pct([c for _,c,_ in v],.5)} motions, timed out median {pct([c for *_,c in v],.5)} (max {max(c for *_,c in v)})")
        # timeouts per route line
        tl = collections.Counter(); seen = collections.Counter(); over = collections.defaultdict(list)
        for r in R:
            for e in r.motions:
                seen[e["line"]] += 1
                tl[e["line"]] += e["timed_out"]
                over[e["line"]].append(e["true_end_err"])
        worst = sorted(tl.items(), key=lambda kc: -kc[1])[:14]
        print("  motions that time out most often (line: timed out/started, median true error at the end):")
        for ln, c in worst:
            if c == 0: continue
            kind, x, y, to = MOT[ln]
            print(f"    line {ln} {kind}({x}, {y}) timeout {to}: {c}/{seen[ln]}, end error median {pct(over[ln],.5):.2f}")


def timing_detail(tag="nonoise_tuned_nocont_120s"):
    """Where the time goes in one run: per motion duration against timeout/1.9, turn settle, waits."""
    ev = events(tag)
    t0 = next(e["t"] for e in ev if e["type"] == "auton_start")
    mot = [e for e in ev if e["type"] == "motion"]
    tot = collections.Counter(); est = collections.Counter(); n = collections.Counter(); to = collections.Counter()
    for e in mot:
        k = e["kind"]
        tot[k] += e["dur_ms"]; est[k] += e["timeout_ms"] / 1.9; n[k] += 1; to[k] += e["timed_out"]
    print(f"== {tag}: motions")
    for k in tot:
        print(f"  {k}: {n[k]} motions, sim {tot[k]/1000:.2f} s vs ATTICUS {est[k]/1000:.2f} s (+{(tot[k]-est[k])/1000:.2f} s, {(tot[k]-est[k])/n[k]:.0f} ms each), timed out {to[k]}")
    end = [e for e in ev if e["type"] in ("end_of_auton", "auton_cut", "route_end")]
    last = max(e["t"] for e in mot) - t0
    print(f"  all motions {sum(tot.values())/1000:.2f} s; last motion ended at {last:.2f} s; everything else (delays, lift waits, grip waits, squares, gaps) {last - sum(tot.values())/1000:.2f} s")
    lg = log(tag)
    for pat in ("grip NOT SEEN", "grip ok", "NOT settled", "creep stopped"):
        print(f"  log lines '{pat}': {sum(1 for _,_,m in lg if pat in m)}")


# ---------------------------------------------------------------- 5 lift
def lift():
    for b, lab in BATCH.items():
        if b == "code_nocont": continue
        ns = collections.Counter(); lines = []
        low = []; high = []; hit = collections.Counter()
        for s in range(1, N + 1):
            tag = f"mc_{b}_seed{s}"
            ev = events(tag)
            last = 0
            for e in ev:
                if e["type"] == "motion": last = e["line"]
                elif e["type"] == "lift_not_settled":
                    k = first_ge(REL, last)
                    ns[PLACE[k].split(" ")[0] if k is not None else "?"] += 1
                    lines.append((s, e["line"]))
                elif e["type"] == "placement" and e["object"] != "nothing held":
                    d = e["bottom_in"] - e["stack_top_in"]
                    if "too low" in e["why"] or "ran into" in e["why"]: low.append((s, e["n"], e["bottom_in"], e["stack_top_in"], e["lift_deg"], e["why"][:40]))
                    if "too high" in e["why"]: high.append((s, e["n"], e["bottom_in"], e["stack_top_in"]))
                elif e["type"] == "stack_hit": hit[e["goal"]] += 1
        print(f"\n== {b} [{lab}] lift")
        print("  [dr4b] NOT settled lines by placement:", dict(ns), "| runs with any:", len(set(s for s, _ in lines)))
        for s, l in lines[:6]: print("    seed", s, l)
        print(f"  releases with the object below the stack top / into the base: {len(low)}; too high: {len(high)}; held object hit a Goal or stack on the way in: {dict(hit)}")
        byn = collections.defaultdict(list)
        for s, n, bo, top, deg, why in low: byn[n].append((bo, top, deg))
        for n in sorted(byn):
            v = byn[n]
            print(f"    release {n}: {len(v)} runs, object bottom median {pct([a for a,_,_ in v],.5):.2f} in vs top {pct([t for _,t,_ in v],.5):.2f} in, lift {pct([d for *_,d in v],.5):.1f} deg")


def lift_settle(tag="nonoise_tuned_nocont_120s"):
    """Settle time after each dr4b::to() target change, from the pose CSV (measured angle within kTolDeg 1.5 of where it ends up)."""
    rows = [r for r in pose(tag) if r["phase"] == "auton"]
    t = [float(r["t"]) for r in rows]; a = [float(r["lift_meas_deg"]) for r in rows]; tr = [float(r["lift_true_deg"]) for r in rows]
    print(f"== {tag}: lift measured angle range {min(a):.1f} to {max(a):.1f} deg, true {min(tr):.1f} to {max(tr):.1f}; max |true - measured| {max(abs(x-y) for x,y in zip(a,tr)):.2f} deg")




def odomplot():
    """sim/out/final/odom_error.png: odometry xy error against time, median / 90th percentile / worst seed, per gain set."""
    import math
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.6), sharex=True)
    for ax, (b, lab) in zip(axs, [(k, v) for k, v in BATCH.items() if k != "code_nocont"]):
        grid = [i * 0.25 for i in range(241)]
        cols = [[] for _ in grid]
        for s in range(1, N + 1):
            rows = [r for r in pose(f"mc_{b}_seed{s}") if r["phase"] == "auton"]
            t0 = float(rows[0]["t"])
            j = 0
            for i, g in enumerate(grid):
                while j + 1 < len(rows) and float(rows[j]["t"]) - t0 < g: j += 1
                r = rows[j]
                cols[i].append(math.hypot(float(r["x"]) - float(r["odom_x"]), float(r["y"]) - float(r["odom_y"])))
        ax.plot(grid, [pct(c, .5) for c in cols], color="#1f5fbf", lw=1.8, label="median of 30 seeds")
        ax.plot(grid, [pct(c, .9) for c in cols], color="#e07b00", lw=1.2, label="90th percentile")
        ax.plot(grid, [max(c) for c in cols], color="#d62020", lw=1.0, ls="--", label="worst seed")
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_ylim(0, 200)
        ax.set_title(f"{b} [{lab}]", fontsize=10)
        ax.set_xlabel("s since markStart()")
        ax.grid(alpha=0.3, which="both")
    axs[0].set_ylabel("odometry xy error, in (true minus arc pose)")
    axs[0].legend(fontsize=8, loc="upper left")
    p = os.path.join(OUT, "odom_error.png")
    fig.savefig(p, dpi=110, bbox_inches="tight")
    print(os.path.abspath(p))


if __name__ == "__main__":
    want = sys.argv[1:] or ["headline", "steps"]
    for w in want: globals()[w]()
