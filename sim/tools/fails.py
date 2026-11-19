"""per seed: every failed grab / placement, in order"""
import glob, json, re, sys
d = sys.argv[1]
for f in sorted(glob.glob(d + "/events_seed*.json"), key=lambda p: int(re.search(r"seed(\d+)", p).group(1))):
    seed = re.search(r"seed(\d+)", f).group(1)
    out = []
    for e in json.load(open(f, encoding="utf-8", errors="replace")):
        t = e["t"] - 2.07
        if e["type"] == "placement" and not e["ok"]:
            out.append("%.0fs place#%d %s al%+.1f ac%+.1f" % (t, e["n"], e["why"][:14], e.get("along_err_in", 9.9), e.get("across_err_in", 9.9)))
        elif e["type"] not in ("placement", "motion", "set_pose", "loose_pushed", "tip_risk", "grab", "auton_start", "loader_present", "clock_check", "toggle_strike") :
            s = json.dumps(e)
            if "tip" in e["type"] or "miss" in e["type"]:
                out.append("%.0fs %s %s lat%s" % (t, e["type"], e.get("object", "")[:22], e.get("lat_in", "")))
    sm = json.load(open(f.replace("events_", "summary_"), encoding="utf-8", errors="replace"))
    print("seed %2s %3d pts tog %s | %s" % (seed, sm["points"], sm["toggle"]["end_state"][:3], "; ".join(out)))
