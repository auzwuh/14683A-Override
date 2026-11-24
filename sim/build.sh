#!/usr/bin/env bash
# Builds sim/out/robot_sim.exe from scratch: the unchanged robot sources plus the
# simulated PROS kernel/devices. Run from Git Bash: sim/build.sh
# Every object is recompiled on every run (nothing is reused from sim/build).
set -euo pipefail
SIM="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SIM/.." && pwd)"
export PATH=/c/msys64/ucrt64/bin:$PATH
CXX=${CXX:-g++}
OBJ="${OBJ:-$SIM/build}"   # object directory (override to prove a clean build)
mkdir -p "$OBJ" "$SIM/out"
REPO_W="$(cygpath -m "$REPO" 2>/dev/null || echo "$REPO")"
CXXFLAGS="-std=gnu++20 -O2 -g0 -w -I$REPO/include -I$REPO/src -I$SIM -include $SIM/sim_printf.h"

# robot code, unchanged (GenSelector replaced by selector_stub.cpp, it needs LVGL)
ROBOT=( "$REPO"/src/main.cpp "$REPO"/src/skills_dr4b.cpp "$REPO"/src/match_autons.cpp "$REPO"/src/gen/*.cpp "$REPO"/src/gen/chassis/*.cpp "$REPO"/src/gen/path_following/*.cpp )
SIMSRC=( "$SIM"/pros_sim.cpp "$SIM"/pros_stubs_gen.cpp "$SIM"/world.cpp "$SIM"/sim_main.cpp "$SIM"/sim_selftest.cpp "$SIM"/selector_stub.cpp )

pids=(); objs=()
for f in "${ROBOT[@]}" "${SIMSRC[@]}"; do
  tu=$(echo "${f#$REPO/}" | tr '/.-' '___')
  o="$OBJ/$tu.o"; objs+=("$o")
  # PROS headers define weak lcd_print / lvgl_init in every TU; MinGW turns those into
  # colliding aliases, so give each TU its own name (neither is ever called).
  $CXX $CXXFLAGS -Dlcd_print=lcd_print_$tu -Dlvgl_init=lvgl_init_$tu -DSIM_REPO_DIR="\"$REPO_W\"" -c "$f" -o "$o" 2> "$o.err" &
  pids+=($!)
done
fail=0
for i in "${!pids[@]}"; do
  if ! wait "${pids[$i]}"; then echo "COMPILE FAILED: ${objs[$i]}"; cat "${objs[$i]}.err"; fail=1; fi
done
[ $fail = 0 ] || exit 1

# --wrap: the sim observes motion start/end, pose writes and motion timeouts without touching robot code
WRAP="-Wl,--wrap=_ZN3arc7Chassis18requestMotionStartEv -Wl,--wrap=_ZN3arc7Chassis9endMotionEv -Wl,--wrap=_ZN3arc7Chassis7setPoseEfffb -Wl,--wrap=_ZN3arc9nudgePoseEdd -Wl,--wrap=_ZN3arc13ExitCondition4timeEji"
$CXX -static -o "$SIM/out/robot_sim.exe" "${objs[@]}" $WRAP
echo "built $SIM/out/robot_sim.exe"
