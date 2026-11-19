#!/bin/bash
# NOTE (18 Nov, route v21): the paths below point at an old session's scratch folder that may be gone.
# Use sim/tools/regen_route.sh instead, it regenerates the route as well and keeps everything in sim/out/runs.
# usage: [SIMFLAGS=..] [GATE=6.0] [REACH=9.5] [TURNEXIT=vel,err,ms] [SEEDS="1 2 .."] run_route.sh ROUTE_OUT_TXT TURN_SCALE TAG
# converts the ATTICUS output, installs it in src/, rebuilds the sim, runs the seeds 8 at a time
PY="/c/Users/USER/Documents/ATTICUS_TERMINAL/.venv/Scripts/python.exe"
T="C:/Users/USER/Documents/ATTICUS_TERMINAL/export/override-skills-dr4b/tools"
S="C:/Users/USER/AppData/Local/Temp/claude/1462e4fb991759cd/0f152982-d759-4e9a-b8d6-e9c9bac43b8a/scratchpad"
REPO="/c/Users/USER/Documents/GitHub/14683A-Override"
src="$1"; scale="$2"; tag="$3"
seeds="${SEEDS:-1 2 3 4 5 6 7 8}"
"$PY" "$T/atticus_to_arc.py" "$src" "$S/regen/$tag.cpp" skillsDr4b "$scale" "${GATE:-6.0}" "${REACH:-9.5}" "${TURNEXIT:-}" >/dev/null || exit 1
cp "$S/regen/$tag.cpp" "$REPO/src/skills_dr4b.cpp"
cd "$REPO" && bash sim/build.sh >/dev/null 2>&1 || { echo "sim build failed"; exit 1; }
mkdir -p "$S/$tag"
export S tag SIMFLAGS
echo $seeds | tr ' ' '\n' | xargs -P 8 -I{} bash -c 'sim/out/robot_sim.exe $SIMFLAGS --seed {} --out "$S/$tag" > "$S/$tag/log{}.txt" 2>&1'
tot=0; n=0; pins=""
for s in $seeds; do
    L="$S/$tag/log$s.txt"
    pts=$(grep -a -o "): [0-9]* points" "$L" | head -1 | grep -o "[0-9]*")
    end=$(grep -a -o "route end [0-9.]* s, [0-9/]* motions timed out" "$L" | head -1)
    [ -z "$QUIET" ] && echo "seed $s: ${pts:-?} pts, $end, placed $(grep -a -c 'place\] #.*: OK' "$L"), place fail $(grep -a -c 'place\] #.*FAIL' "$L"), empty $(grep -a -c 'claw empty' "$L"), tipped $(grep -a -c 'grab\].*TIPPED by' "$L")"
    tot=$((tot + ${pts:-0})); n=$((n + 1)); pins="$pins ${pts:-?}"
done
echo "$tag: mean $((tot / n)) pts over $n seeds |$pins"
