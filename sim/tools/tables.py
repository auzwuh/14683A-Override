"""Collect sim/out results into markdown tables (stdout). Run by sim/run.sh.
One row per 20-seed batch (aggregate_20_<tag>.json) and per single run (summary_<tag>.json)."""
import glob, json, os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")


def m(agg, key, f="mean"):
    v = agg["metrics"].get(key)
    return v[f] if v else float("nan")


print("## 20-seed batches (ranges matter more than means: most plant constants are ASSUMED)\n")
print("| batch | points mean | min..max | points histogram | Pins seated | motions timed out | route done in 60 s | "
      "odom xy err final in (mean / max) | heading err final deg (mean / max) | lost (>45 deg) | IMU reads over gate | "
      "stack hits | dropped | grabs ok / miss / tipped | Midfield | crashed |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for p in sorted(glob.glob(os.path.join(OUT, "aggregate_20_*.json"))):
    a = json.load(open(p))
    tag = os.path.basename(p)[len("aggregate_20_"):-len(".json")]
    hist = " ".join(f"{k}:{v}" for k, v in a["points_distribution"].items())
    print(f"| {tag} | {m(a,'points'):.1f} | {m(a,'points','min'):.0f}..{m(a,'points','max'):.0f} | {hist} | {m(a,'pins'):.2f} | "
          f"{m(a,'motions_timed_out'):.1f} | {m(a,'completed'):.2f} | {m(a,'odom_err_final'):.1f} / {m(a,'odom_err_final','max'):.1f} | "
          f"{m(a,'head_err_final'):.1f} / {m(a,'head_err_final','max'):.1f} | {m(a,'lost_heading_over_45'):.2f} | "
          f"{m(a,'imu_reads_over_gate'):.0f} | {m(a,'stack_hits'):.1f} | {m(a,'dropped'):.1f} | "
          f"{m(a,'grab_ok'):.1f} / {m(a,'grab_miss'):.1f} / {m(a,'tipped'):.1f} | {m(a,'midfield'):.2f} | "
          f"{len(a['crashed_or_watchdog_seeds'])} |")

print("\n## Single runs\n")
print("| run | points | placements ok / releases | route end s | motions timed out | odom xy err max / final in | heading err final deg | "
      "stop resets applied | mount readings ok / rejected | squares | track corrections | IMU reads over gate | speed at cut in/s | Midfield at rest / at cut |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for p in sorted(glob.glob(os.path.join(OUT, "summary_*.json"))):
    tag = os.path.basename(p)[len("summary_"):-len(".json")]
    if tag.startswith("mc_"):
        continue
    s = json.load(open(p))
    if "at_60s_cut" not in s:   # written by an older build of the sim
        continue
    ok = sum(1 for x in s["placements"] if x["ok"])
    r, o, c = s["resets"], s["odometry"], s["at_60s_cut"]
    end = "not finished" if s["route_end_s"] < 0 else f"{s['route_end_s']:.2f}"
    print(f"| {tag} | {s['points']} | {ok} / {len(s['placements'])} | {end} | {s['motions']['timed_out']} of {s['motions']['count']} | "
          f"{o['xy_err_max_in']:.2f} / {o['xy_err_final_in']:.2f} | {o['heading_err_final_deg']:.2f} | {r['stop_resets_applied']} | "
          f"{r['mount_readings_ok']} / {r['mount_readings_rejected']} | {r['squares']} | {r['continuous_track_corrections']} | "
          f"{s['imu_fusion']['reads_over_the_disagreement_gate']} of {s['imu_fusion']['robot_reads_of_both_sensors']} | "
          f"{c['true_speed_in_s']:.1f} | {str(s['midfield_at_end']).lower()} / {str(c['midfield_at_cut']).lower()} |")
