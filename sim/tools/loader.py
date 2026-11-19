"""Loader visits: when, how long each part takes, how long the match loader has between grabs"""
import glob, json, re, sys, statistics as st, collections
d = sys.argv[1]
T = collections.defaultdict(lambda: collections.defaultdict(list))
lat = collections.defaultdict(list)
tips = collections.Counter()
for f in glob.glob(d + "/events_seed*.json"):
    ev = json.load(open(f, encoding="utf-8", errors="replace"))
    mot = [e for e in ev if e["type"] == "motion"]
    for i, e in enumerate(mot):
        if e["kind"] == "reachTo" and abs(abs(e["target"][0]) - 58.8) < 0.1 and abs(e["target"][1] - 68.38) < 0.1 and not e["cancelled"]:
            n = sum(1 for m in mot[:i + 1] if m["kind"] == "reachTo" and abs(abs(m["target"][0]) - 58.8) < 0.1 and not m["cancelled"])
            arrive = e["t"] - 2.07
            turn = next(m for m in reversed(mot[:i]) if m["kind"].startswith("turn"))
            back_in = next(m for m in reversed(mot[:i]) if m["kind"] == "moveToPoint")
            nxt = mot[i + 1] if i + 1 < len(mot) else None
            T[n]["arrive"].append(arrive)
            T[n]["drive_to_pivot"].append(back_in["dur_ms"] / 1000)
            T[n]["turn"].append(turn["dur_ms"] / 1000)
            T[n]["reach"].append(e["dur_ms"] / 1000)
            if nxt:
                T[n]["dwell"].append(nxt["t"] - nxt["dur_ms"] / 1000 - e["t"])
                T[n]["back_out"].append(nxt["dur_ms"] / 1000)
    for e in ev:
        if e["type"] == "grab" and "Loader" in e.get("object", ""):
            lat[e["object"][:22]].append(e["lat_in"])
        if e["type"] == "tipped" and "Loader" in e.get("object", ""):
            tips[e["object"][:22]] += 1
names = {1: "top 1 (Cup+Pin -> G1)", 2: "top 2 (Cup+Pin -> G1)", 3: "top 3 (Cup+Pin -> G1)", 4: "top 4 (Pin -> Toggle, G2)", 5: "bottom 1 (Cup+Pin -> G2)", 6: "bottom 2 (Pin -> A2)", 7: "bottom 3 (Pin -> Tall)"}
prev = None
for n in sorted(T):
    a = st.mean(T[n]["arrive"])
    print("%-27s claw at the Loader %5.1f s (%.1f to %.1f)%s | pivot move %.2f turn %.2f drive in %.2f dwell %.2f back out %.2f" % (
        names.get(n, n), a, min(T[n]["arrive"]), max(T[n]["arrive"]), "  +%.1f s after the last" % (a - prev) if prev and n not in (5,) else "",
        *(st.mean(T[n][k]) for k in ("drive_to_pivot", "turn", "reach", "dwell", "back_out"))))
    prev = a
allv = [abs(v) for vs in lat.values() for v in vs]
print("grab offset |lat| mean %.2f, over 0.6 in: %d of %d, tipped: %d" % (st.mean(allv), sum(v > 0.6 for v in allv), len(allv), sum(tips.values())))
