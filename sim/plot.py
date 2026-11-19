"""Draw one sim run: true path vs odometry path on the field, with Goals, Loaders,
resets and placements. usage: python sim/plot.py [tag ...]   (default: seed1)
Reads sim/out/pose_<tag>.csv and events_<tag>.json, writes sim/out/route_<tag>.png."""
import csv, json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
H = 140.41 / 2
GOALS = [(23.40, 46.80, "S"), (46.80, 23.40, "S"), (-46.80, -23.40, "S"), (-23.40, -46.80, "S"),
         (-23.40, 46.80, "A"), (-46.80, 23.40, "A"), (46.80, -23.40, "A"), (23.40, -46.80, "A"), (0.0, 0.0, "T")]
LOADERS = [(58.80, 1), (-58.80, 1), (58.80, -1), (-58.80, -1)]


def plot(tag):
    rows = list(csv.DictReader(open(os.path.join(OUT, f"pose_{tag}.csv"))))
    events = json.load(open(os.path.join(OUT, f"events_{tag}.json")))
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.add_patch(Rectangle((-H, -H), 2 * H, 2 * H, fill=False, lw=2, color="0.2"))
    ax.add_patch(Polygon([(0, 23.4), (23.4, 0), (0, -23.4), (-23.4, 0)], closed=True, color="#f3e6b3", alpha=0.6, label="Midfield"))
    for x, y, k in GOALS:
        ax.add_patch(Circle((x, y), 3.22, color={"S": "#9a9a9a", "A": "#d06060", "T": "#404040"}[k], alpha=0.8))
        ax.text(x, y - 5.5, {"S": "Short", "A": "Alliance", "T": "Tall"}[k], ha="center", fontsize=7)
    for x, s in LOADERS:
        y0 = 66.47 if s > 0 else -H
        ax.add_patch(Rectangle((x - 2.42, y0), 4.84, 3.74, color="#7a5c2e"))
    for ph, style, lab in (("auton", "-", "true path"),):
        xs = [float(r["x"]) for r in rows if r["phase"] == ph]
        ys = [float(r["y"]) for r in rows if r["phase"] == ph]
        ax.plot(xs, ys, style, color="#1f5fbf", lw=1.6, label=lab)
    ox = [float(r["odom_x"]) for r in rows if r["phase"] == "auton"]
    oy = [float(r["odom_y"]) for r in rows if r["phase"] == "auton"]
    ax.plot(ox, oy, "--", color="#e07b00", lw=1.2, label="odometry path")
    dx = [float(r["x"]) for r in rows if r["phase"] == "driver"]
    dy = [float(r["y"]) for r in rows if r["phase"] == "driver"]
    if dx:
        ax.plot(dx, dy, ":", color="#2a9d2a", lw=1.4, label="driver scenario (true)")
    first = {}
    for e in events:
        t = e.get("type")
        if t == "set_pose" and e["source"] in ("stop_reset", "square"):
            x, y = e["true"][0], e["true"][1]
            m = "^" if e["source"] == "stop_reset" else "s"
            lab = None if e["source"] in first else ("stop reset applied" if m == "^" else "bumper square")
            first[e["source"]] = 1
            ax.plot(x, y, m, ms=8, mfc="none", mec="#7b2cbf", mew=1.6, label=lab)
        elif t == "placement" and "x" in e:
            ok = e["ok"]
            lab = None if ("p", ok) in first else ("placement OK" if ok else "placement FAIL")
            first[("p", ok)] = 1
            ax.plot(e["x"], e["y"], "o" if ok else "x", ms=9, color="#1a9e1a" if ok else "#d62020", mew=2, label=lab)
            ax.annotate(str(e["n"]), (e["x"], e["y"]), xytext=(4, 4), textcoords="offset points", fontsize=8)
        elif t in ("grab", "grab_miss"):
            pass
    summ = json.load(open(os.path.join(OUT, f"summary_{tag}.json")))
    o = summ["odometry"]
    m = summ["motions"]
    ax.set_title(f"{tag} ({summ['drive_gains']}): {summ['points']} pts, {summ['pins_seated']} Pins seated, "
                 f"midfield {summ['midfield_at_end']}, route end {summ['route_end_s']} s, {m['timed_out']}/{m['count']} motions timed out" + chr(10) +
                 f"odometry xy error max {o['xy_err_max_in']:.2f} in, final {o['xy_err_final_in']:.2f} in", fontsize=10)
    ax.set_xlim(-H - 3, H + 3)
    ax.set_ylim(-H - 3, H + 3)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x (in), +x right")
    ax.set_ylabel("y (in), +y = red wall")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    path = os.path.join(OUT, f"route_{tag}.png")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    for tag in sys.argv[1:] or ["seed1"]:
        plot(tag)
