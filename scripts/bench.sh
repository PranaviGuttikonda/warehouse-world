#!/bin/sh
# Given: a built ./build/warehouse. Approach: run every robot count over
# seeds 1-5 headless and print README table rows: mean, then min-max.
# Usage: scripts/bench.sh [extra warehouse args, e.g. --planner stastar]
set -eu
BIN=${BIN:-./build/warehouse}
TICKS=${TICKS:-20000}
COUNTS=${COUNTS:-"4 8 12 16 20 24 28 32 40"}
SEEDS=${SEEDS:-"1 2 3 4 5"}

means="" ranges=""
for n in $COUNTS; do
    vals=""
    for s in $SEEDS; do
        v=$("$BIN" --headless --robots "$n" --ticks "$TICKS" --seed "$s" "$@" |
            awk '/^deliveries/ {print $2}')
        vals="$vals $v"
    done
    stats=$(echo "$vals" | tr ' ' '\n' | awk 'NF {s += $1; if (min == "" || $1 < min) min = $1;
        if ($1 > max) max = $1; c++} END {printf "%d %d-%d", s / c + 0.5, min, max}')
    means="$means | ${stats%% *}"
    ranges="$ranges | ${stats#* }"
done
echo "| Robots | $(echo $COUNTS | sed 's/ / | /g') |"
echo "|---|$(for _ in $COUNTS; do printf -- '---|'; done)"
echo "| Mean$means |"
echo "| Min-max$ranges |"
