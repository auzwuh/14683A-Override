#!/usr/bin/env bash
# Standard scenarios. Usage: sim/run.sh [extra robot_sim flags]
# Builds, runs the selftest (code gains and --tuned-drive), then the route, then the
# sensitivity sweeps. Outputs land in sim/out/, the collected tables in sim/out/tables.md.
# Every sim constant is in sim/sim_config.hpp; the ones marked ASSUMED are not measured and
# the sweeps below show how far the results move with them. Quote ranges, not single numbers.
set -uo pipefail
SIM="$(cd "$(dirname "$0")" && pwd)"
PY=${PY:-/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe}
"$SIM/build.sh" || exit 1
EXE="$SIM/out/robot_sim.exe"
mc() { local tag=$1; shift; "$EXE" --runs 20 --tag "$tag" "$@" > /dev/null; }   # 20 seeds -> aggregate_20_<tag>.json

"$EXE" --selftest --quiet "$@"
"$EXE" --selftest --tuned-drive --quiet "$@"

# ---- robot code exactly as written. --no-noise is ONE sample of an ideal plant, not the reference result.
"$EXE" --no-noise --quiet "$@"
"$EXE" --seed 1 --quiet "$@"
"$EXE" --seed 1 --driver --quiet "$@"
# opcontrol against a test motion left running (auton cut during a delay, not inside a motion)
"$EXE" --seed 1 --driver --auton-ms 2100 --tag seed1_driver_leftover --quiet "$@"
# the two start assumptions, side by side: rear contact 8.4 in (CAD) and 7.01 in (what the route assumes)
mc code "$@"
mc code_rear701 --rear 7.01 "$@"

# ---- experiments that isolate the two main robot findings
#      --tuned-drive: drive PID gains that work on the ASSUMED plant.  --no-cont-track: continuous
#      wall tracking off (every chassis.setPose() drops one odometry tick, odom.cpp).
"$EXE" --no-noise --tuned-drive --quiet "$@"
"$EXE" --no-noise --no-cont-track --quiet "$@"
T="--tuned-drive --no-cont-track"
"$EXE" --no-noise $T --quiet "$@"
"$EXE" --seed 1 $T --quiet "$@"
mc tuned_nocont $T "$@"
mc tuned_nocont_rear701 $T --rear 7.01 "$@"
mc tuned_nocont_drawh $T --draw-heights "$@"                 # Short Goal base 5.77 in (draw.py) instead of the code's 3.25
mc tuned_nocont_rear701_drawh $T --rear 7.01 --draw-heights "$@"
# the two IMU assumptions: samples of both sensors arriving together, and scales measured
mc tuned_nocont_imusync $T --imu-jitter 0 "$@"
mc tuned_nocont_imucal $T --imu-gain 1.0 "$@"
mc tuned_nocont_imusync_imucal $T --imu-jitter 0 --imu-gain 1.0 "$@"

# ---- scenario switches on top of that, one ideal-plant run each (F2 F3 F5 F7 F8 F9 F10 of the parameter sheet)
"$EXE" --no-noise $T --quiet --rear 7.01 "$@"
"$EXE" --no-noise $T --quiet --rear 7.01 --goal-pins "$@"
"$EXE" --no-noise $T --quiet --draw-heights "$@"
"$EXE" --no-noise $T --quiet --wheel-dia 2.125 "$@"
"$EXE" --no-noise $T --quiet --imu-gain 0.989 "$@"
"$EXE" --no-noise $T --quiet --loader-x 58.505 "$@"
"$EXE" --no-noise $T --quiet --no-clusters "$@"
"$EXE" --no-noise $T --quiet --imu-jitter 1 --imu-phase-b 0 "$@"   # IMU B's samples land +-1 ms around the odometry read
"$EXE" --no-noise $T --quiet --imu-b-missing "$@"
"$EXE" --no-noise $T --quiet --imu-b-drop 20 "$@"
"$EXE" --no-noise $T --quiet --auton-ms 90000 --tag info_90s "$@"

# ---- sensitivity to constants nobody has measured (each: 20 seeds with that one constant fixed)
for v in 0.25 0.5 1.0 2.0; do                      # turning scrub: turn overshoot and the tuned gains hinge on it
  mc code_scrub$v --scrub $v "$@"
  mc tuned_nocont_scrub$v $T --scrub $v "$@"
  "$EXE" --selftest --quiet --scrub $v --tag selftest_scrub$v "$@" 2> /dev/null
done
for v in 0.5 1.9 3.8; do                           # drivetrain drag: where the code's lateral P loop stalls
  mc code_dfric$v --drive-fric $v "$@"
  mc tuned_nocont_dfric$v $T --drive-fric $v "$@"
  "$EXE" --selftest --quiet --drive-fric $v --tag selftest_dfric$v "$@" 2> /dev/null
done
R="$T --rear 7.01"                                  # claw / Goal tolerances, on the better start assumption
for k in "--grab-lat 0.6" "--grab-lat 1.02" "--grab-ahead 1.4" "--grab-ahead 2.1" "--release-ms 250" "--release-fwd 0" \
         "--capture-r 0.75" "--capture-r 1.5" "--pitch 3.32" "--pitch 3.55" "--lift-slop 0" "--lift-slop 2" "--lift-err 2"; do
  mc "tuned_nocont_rear701_$(echo "$k" | tr -d ' -')" $R $k "$@"
done

"$PY" "$SIM/plot.py" nonoise seed1 nonoise_tuned nonoise_nocont nonoise_tuned_nocont seed1_tuned_nocont
"$PY" "$SIM/tools/tables.py" | tee "$SIM/out/tables.md"
