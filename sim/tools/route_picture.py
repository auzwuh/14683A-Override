"""Picture of one simulated skills run for people, not for debugging: where the robot drives,
coloured by time, with every pickup and placement numbered in order.

usage: python sim/tools/route_picture.py TAG [out.png]
Reads sim/out/pose_TAG.csv, events_TAG.json, summary_TAG.json (make them with
sim/out/robot_sim.exe --calibrated --seed N --tag TAG --quiet).
"""
import csv, json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle, Rectangle, Polygon

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out")
tag = sys.argv[1]
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(OUT, f"skills_run_{tag}.png")
H = 140.41 / 2
T0 = 2.07   # autonomous starts this long after the sim starts

rows = [r for r in csv.DictReader(open(os.path.join(OUT, f"pose_{tag}.csv"))) if r["phase"] == "auton"]
events = json.load(open(os.path.join(OUT, f"events_{tag}.json")))
summ = json.load(open(os.path.join(OUT, f"summary_{tag}.json")))

fig, ax = plt.subplots(figsize=(16.5, 10.5))
ax.add_patch(Rectangle((-H, -H), 2 * H, 2 * H, fill=True, fc="#f4f4f1", ec="#333333", lw=2.5, zorder=0))
for k in range(1, 6):   # tile lines
    v = -H + k * 2 * H / 6
    ax.plot([v, v], [-H, H], color="#dcdcd6", lw=0.8, zorder=0.5)
    ax.plot([-H, H], [v, v], color="#dcdcd6", lw=0.8, zorder=0.5)
ax.add_patch(Polygon([(0, 23.4), (23.4, 0), (0, -23.4), (-23.4, 0)], closed=True, fc="#f6e7a8", ec="#c9b458", alpha=0.75, zorder=1))
ax.text(0, -15.5, "Midfield", ha="center", fontsize=9, color="#8a7420", zorder=2)
# walls
ax.plot([-H, H], [H, H], color="#c0392b", lw=7, solid_capstyle="butt", zorder=2)
ax.text(0, H + 3.2, "RED WALL (your side)", ha="center", fontsize=12, color="#c0392b", weight="bold")
ax.plot([-H, H], [-H, -H], color="#2c5fa8", lw=7, solid_capstyle="butt", zorder=2)
ax.text(0, -H - 5.2, "BLUE WALL", ha="center", fontsize=12, color="#2c5fa8", weight="bold")
# Toggle on the red wall, Loaders
ax.add_patch(Rectangle((-13, H - 2.0), 26, 2.0, fc="#d86bd8", ec="#8a2f8a", zorder=3))
ax.text(0, H - 5.6, "Toggle", ha="center", fontsize=9, color="#8a2f8a", weight="bold", zorder=6)
for x, y0, name in ((58.80, H - 3.74, "Loader\n(start side)"), (-58.80, H - 3.74, "Loader\n(far side)"), (58.80, -H, ""), (-58.80, -H, "")):
    ax.add_patch(Rectangle((x - 2.42, y0), 4.84, 3.74, fc="#8a6a34", ec="#4d3a17", zorder=3))
    if name:
        ax.text(x + (6.0 if x > 0 else -6.0), H + 3.2, name, ha="center", va="center", fontsize=8.5, color="#4d3a17", weight="bold")
# Goals, with how many Pins ended on each
pins = {}
for g in summ["goals"]:
    pins[g["name"]] = g["pins_seated"]
GOALS = [(23.40, 46.80, "Short Goal", "#8d8d8d", "Short G1 (24,96)"), (46.80, 23.40, "Short", "#b9b9b9", None),
         (-46.80, -23.40, "Short", "#b9b9b9", None), (-23.40, -46.80, "Short", "#b9b9b9", None),
         (-23.40, 46.80, "Alliance Goal", "#d0584f", "Alliance G2 (24,48)"), (-46.80, 23.40, "2nd Alliance Goal", "#d0584f", "Alliance (48,24)"),
         (46.80, -23.40, "blue", "#7f9fd1", None), (23.40, -46.80, "blue", "#7f9fd1", None), (0.0, 0.0, "Tall Goal", "#3d3d3d", "Tall (72,72)")]
for x, y, name, col, key in GOALS:
    ax.add_patch(Circle((x, y), 3.22, fc=col, ec="#222222", lw=1, zorder=4))
    if key:
        n = pins.get(key, 0)
        lx, ly, va = (x + 13.5, y + 7.0, "center") if key.startswith("Tall") else (x, y + 5.0, "bottom") if y > 40 else (x, y - 5.2, "top")
        ax.text(lx, ly, f"{name}\n{n} Pin{'s' if n != 1 else ''}", ha="center", va=va, fontsize=8.5, weight="bold", zorder=6,
                bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    else:
        ax.text(x, y - 5.6, name, ha="center", va="top", fontsize=7, color="#666666")
# floor Cup+Pins the robot picks up
for x, y in ((-23.40, 23.40), (-46.80, 46.80), (23.40, -23.40), (46.80, -46.80)):
    ax.add_patch(Circle((x, y), 1.9, fc="#ffd84d", ec="#8a6d00", lw=1.2, zorder=4))

# path, coloured by time
xs = [float(r["x"]) for r in rows]
ys = [float(r["y"]) for r in rows]
ts = [float(r["t"]) - T0 for r in rows]
step = 3
pts = list(zip(xs[::step], ys[::step]))
segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
lc = LineCollection(segs, cmap="turbo", array=ts[::step][:-1], lw=2.6, zorder=5, capstyle="round")
lc.set_clim(0, 60)
ax.add_collection(lc)
cb = fig.colorbar(lc, ax=ax, fraction=0.025, pad=0.01)
cb.set_label("time in the run (s)")
# start
ax.plot(xs[0], ys[0], marker="s", ms=13, mfc="white", mec="black", mew=2, zorder=7)
ax.annotate("START\nback on the wall", (xs[0], ys[0]), xytext=(xs[0] - 15, ys[0] + 2.5), fontsize=9, weight="bold", ha="center",
            arrowprops=dict(arrowstyle="-", color="black", lw=1), zorder=8)

# numbered steps: every pickup and every placement, in time order
steps = []
for e in events:
    t = e["t"] - T0
    if e["type"] == "grab":
        steps.append((t, "pick", e.get("object", "")))
    elif e["type"] == "placement":
        steps.append((t, "place" if e["ok"] else "miss", e["goal"], e.get("object", "")))
    elif e["type"] == "toggle_flip_start":
        steps.append((t, "toggle"))
steps.sort()


def pos_at(t):
    i = min(range(len(ts)), key=lambda k: abs(ts[k] - t))
    return xs[i], ys[i]


def short_obj(o):
    if "Loader" in o:
        return ("Cup+Pin" if "Cup+Pin" in o else "bare Pin") + (", start-side Loader" if o.startswith("top") else ", far-side Loader")
    return "floor Cup+Pin"


def short_goal(g):
    return {"Short G1 (24,96)": "Short Goal", "Alliance G2 (24,48)": "Alliance Goal", "Alliance (48,24)": "2nd Alliance Goal", "Tall (72,72)": "Tall Goal"}.get(g, g)


lines = []
placed = 0
used = {}
for n, s in enumerate(steps, 1):
    t = s[0]
    x, y = pos_at(t)
    key = (round(x / 5), round(y / 5))          # fan out numbers that land on the same spot
    k = used.get(key, 0)
    used[key] = k + 1
    ox, oy = [(0, 0), (4.2, 0), (-4.2, 0), (0, 4.2), (0, -4.2), (4.2, 4.2), (-4.2, 4.2), (4.2, -4.2)][k % 8]
    col = {"pick": "#1f6fd0", "place": "#178a2d", "miss": "#c62828", "toggle": "#8a2f8a"}[s[1]]
    ax.text(x + ox, y + oy, str(n), ha="center", va="center", fontsize=8, color="white", weight="bold", zorder=9,
            bbox=dict(boxstyle="circle,pad=0.22", fc=col, ec="white", lw=0.8))
    if s[1] == "pick":
        lines.append((col, f"{n:>2}  {t:4.1f} s  pick up {short_obj(s[2])}"))
    elif s[1] == "toggle":
        lines.append((col, f"{n:>2}  {t:4.1f} s  spin the Toggle to red"))
    else:
        placed += s[1] == "place"
        lines.append((col, f"{n:>2}  {t:4.1f} s  {'place on' if s[1] == 'place' else 'MISSED'} {short_goal(s[2])}" + (f"   (Pin {placed})" if s[1] == "place" else "")))

ax.set_xlim(-H - 12, H + 12)
ax.set_ylim(-H - 9, H + 8)
ax.set_aspect("equal")
ax.axis("off")
ax.set_title(f"Skills route v21, simulated run: {summ['points']} points ({summ['pins_seated']} Pins + Midfield), "
             f"last release at {max(p['t'] for p in summ['placements']) - T0:.1f} s", fontsize=12, weight="bold", pad=22)
# step list on the right
fig.subplots_adjust(left=0.01, right=0.69)
fx, fy = 0.725, 0.90
fig.text(fx, fy + 0.025, "What happens, in order", fontsize=12, weight="bold")
for i, (col, txt) in enumerate(lines):
    fig.text(fx, fy - i * 0.0355, txt, fontsize=9.0, color=col, family="monospace")
fig.text(fx, fy - (len(lines) + 0.6) * 0.0355, "blue = pick up   green = place   purple = Toggle\nyellow dots = floor Cup+Pins   path colour = time",
         fontsize=8.5, color="#444444", va="top")
fig.savefig(dst, dpi=115)
print(dst)
