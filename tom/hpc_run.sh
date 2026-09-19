#!/bin/bash
# Runs N headless Webots instances inside the container. Safe on shared nodes:
# Xvfb picks a free display itself, and each instance gets its own port block.
# usage: hpc_run.sh N SIM_SECONDS WORLD OUTDIR TAG
N=${1:-4}; SIMS=${2:-60}; WORLD=${3:-bench_A}; OUT=${4:-/results}; TAG=${5:-run}
WORK=${TMPDIR:-/tmp}/wbmc_$$; mkdir -p $WORK
export HOME=$WORK
exec 3<> $WORK/displayfd
Xvfb -displayfd 3 -screen 0 320x240x24 -nolisten tcp >/dev/null 2>&1 &
XPID=$!
for _ in $(seq 1 50); do D=$(head -1 $WORK/displayfd 2>/dev/null); [ -n "$D" ] && break; sleep 0.2; done
export DISPLAY=:$D
PORT0=$(( 20000 + (RANDOM % 4000) * 10 ))
pids=(); start=$(date +%s.%N)
for i in $(seq 1 $N); do
  ( export WEBOTS_TMPDIR=$WORK/wb$i; mkdir -p $WEBOTS_TMPDIR
    export BENCH_SIM_SECONDS=$SIMS BENCH_OUT=$OUT/${TAG}_$i.json
    webots --port=$((PORT0 + i * 3)) --batch --mode=fast --no-rendering --stdout --stderr \
      /proj/worlds/$WORLD.wbt > $OUT/${TAG}_$i.log 2>&1 ) &
  pids+=($!)
done
for p in "${pids[@]}"; do wait $p; done
end=$(date +%s.%N); kill $XPID 2>/dev/null; rm -rf $WORK
echo "HOST=$(hostname) DISPLAY=$DISPLAY N=$N SIM=$SIMS WORLD=$WORLD TOTAL_WALL=$(echo "$end - $start" | bc)"
