"""usage: pose_err.py pose_csv [t0 t1 step]  -> true vs odometry pose over time"""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
t0, t1, step = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (2.1, 60.0, 2.0)
prev = None
for r in rows:
    t = float(r['t'])
    if t < t0 or t > t1: continue
    if prev is None or t - prev >= step:
        prev = t
        x, y, th = float(r['x']), float(r['y']), float(r['theta'])
        ox, oy, oth = float(r['odom_x']), float(r['odom_y']), float(r['odom_theta'])
        dh = (oth - th + 180) % 360 - 180
        print("t %5.2f true (%6.2f %6.2f %6.1f) odom-true dxy (%+5.2f %+5.2f) dh %+5.1f  v %5.1f w %6.1f" % (t, x, y, th, ox - x, oy - y, dh, float(r['v_in_s']), float(r['w_deg_s'])))
