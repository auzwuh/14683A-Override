"""usage: an.py DIR [DIR ...]   summary of every summary_seed*.json in each run directory"""
import glob, json, re, sys, collections

for d in sys.argv[1:]:
    rows = []
    for f in sorted(glob.glob(d + "/summary_seed*.json"), key=lambda p: int(re.search(r"seed(\d+)", p).group(1))):
        try:
            rows.append(json.load(open(f, encoding="utf-8", errors="replace")))
        except Exception as e:
            print("bad", f, e)
    if not rows:
        print(d, "no summaries"); continue
    n = len(rows)
    pts = [r["points"] for r in rows]
    pins = [r["pins_seated"] for r in rows]
    tog = sum(1 for r in rows if r["toggle"]["end_state"] == "red")
    mid = sum(1 for r in rows if r["midfield_at_end"])
    why = collections.Counter()
    nplace = collections.Counter(); okplace = collections.Counter(); tplace = collections.defaultdict(list)
    for r in rows:
        for p in r["placements"]:
            nplace[p["n"]] += 1
            tplace[p["n"]].append(p["t"] - 2.07)
            if p["ok"]: okplace[p["n"]] += 1
            else: why["#%d %s" % (p["n"], p["why"][:28])] += 1
    g = collections.Counter()
    for r in rows:
        for k in ("miss", "tipped"):
            g[k] += r["grabs"][k]
    print("%s: n %d | points mean %.1f min %d max %d | 128: %d, >=118: %d | pins mean %.2f | toggle red %d/%d | midfield %d/%d | grabs miss %d tipped %d | route end mean %.1f max %.1f"
          % (d.split("/")[-1], n, sum(pts) / n, min(pts), max(pts), sum(p >= 128 for p in pts), sum(p >= 118 for p in pts), sum(pins) / n, tog, n, mid, n,
             g["miss"], g["tipped"], sum(r["route_end_s"] for r in rows) / n, max(r["route_end_s"] for r in rows)))
    print("   placed ok/tried, mean t: " + "  ".join("#%d %d/%d %.1f" % (k, okplace[k], nplace[k], sum(tplace[k]) / len(tplace[k])) for k in sorted(nplace)))
    if why:
        print("   fails: " + ", ".join("%s x%d" % kv for kv in why.most_common(8)))
