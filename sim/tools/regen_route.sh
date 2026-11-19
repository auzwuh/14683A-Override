#!/bin/bash
# usage: [GENFLAGS=..] [CONVARGS="1.0 6.0 9.5 10,1.5,80 --flags"] [SIMFLAGS=..] [SEEDS="1 2 .."] [QUIET=1] sim/tools/regen_route.sh TAG
# gen_route.py -> ATTICUS compile -> atticus_to_arc.py -> src/skills_dr4b.cpp -> sim build -> seeds 8 at a time (default --calibrated, seeds 1 to 16)
# results in sim/out/runs/TAG, then: python sim/tools/summary.py sim/out/runs/TAG ; python sim/tools/fails.py sim/out/runs/TAG
PY="/c/Users/USER/Documents/ATTICUS_TERMINAL/.venv/Scripts/python.exe"
AT="C:/Users/USER/Documents/ATTICUS_TERMINAL"
T="$AT/export/override-skills-dr4b/tools"
REPO="/c/Users/USER/Documents/GitHub/14683A-Override"
S="$REPO/sim/out/runs"
mkdir -p "$S/regen"
tag="$1"; seeds="${SEEDS:-1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16}"
"$PY" "$T/gen_route.py" "$S/regen/$tag.json" $GENFLAGS > "$S/regen/$tag.gen.log" 2>&1 || { echo "gen_route failed"; tail -5 "$S/regen/$tag.gen.log"; exit 1; }
"$PY" "$T/harness.py" "$AT" "$S/regen/$tag.json" "$S/regen/${tag}_out.txt" > "$S/regen/$tag.har.log" 2>&1 || { echo "harness failed"; tail -5 "$S/regen/$tag.har.log"; exit 1; }
grep TOTAL_S "$S/regen/$tag.har.log"
"$PY" "$T/atticus_to_arc.py" "$S/regen/${tag}_out.txt" "$S/regen/$tag.cpp" skillsDr4b ${CONVARGS:-} > "$S/regen/$tag.conv.log" 2>&1 || { echo "convert failed"; tail -5 "$S/regen/$tag.conv.log"; exit 1; }
cp "$S/regen/$tag.cpp" "$REPO/src/skills_dr4b.cpp"
cd "$REPO" && bash sim/build.sh > "$S/regen/$tag.build.log" 2>&1 || { echo "sim build failed"; tail -20 "$S/regen/$tag.build.log"; exit 1; }
rm -rf "$S/$tag"; mkdir -p "$S/regen" "$S/$tag"
export S tag SIMFLAGS
echo $seeds | tr ' ' '\n' | xargs -P 8 -I{} bash -c 'sim/out/robot_sim.exe ${SIMFLAGS:---calibrated} --seed {} --out "$S/$tag" > "$S/$tag/log{}.txt" 2>&1'
tot=0; n=0; pins=""
for s in $seeds; do
    L="$S/$tag/log$s.txt"
    pts=$(grep -a -o "): [0-9]* points" "$L" | head -1 | grep -o "[0-9]*")
    end=$(grep -a -o "route end [0-9.]* s, [0-9/]* motions timed out" "$L" | head -1)
    [ -z "$QUIET" ] && echo "seed $s: ${pts:-?} pts, $end, placed $(grep -a -c 'place\] #.*: OK' "$L"), fail $(grep -a -c 'place\] #.*FAIL' "$L"), empty $(grep -a -c 'nothing in the claw' "$L"), tipped $(grep -a -c 'grab\].*TIPPED' "$L"), last release $(grep -a -o '\[t [0-9. ]*\] release' "$L" | tail -1)"
    tot=$((tot + ${pts:-0})); n=$((n + 1)); pins="$pins ${pts:-?}"
done
echo "$tag: mean $((tot / n)) pts over $n seeds |$pins"
