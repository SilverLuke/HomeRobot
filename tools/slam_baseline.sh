#!/usr/bin/env bash
# Official SLAM baseline protocol (docs/specs/icp-slam-rework-plan.md T4/T5):
# one pattern run with FRESH state — new sim, empty map, new trace — so runs
# are comparable and repeatable.
#
# Run ON the Linux build machine, from the repo root, inside nix-shell:
#   nix-shell --run "./tools/slam_baseline.sh p2 r1"
#   nix-shell --run "./tools/slam_baseline.sh all"      # p1..p5 x r1..r3
#
# Patterns: p1 p2 p3 (plain), p4 (= p2 with HR_FAULT_RIGHT_ENCODER=1),
# p5 (out - server kill/restart - back). Results land in
# logs/bench/<pattern>-<run>/metrics.json; aggregate with
#   python3 tools/slam_benchmark.py summarize logs/bench/*/metrics.json
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
SERVER_BIN="$ROOT/server/target/debug/server"
export HR_CMD_SENDER="$ROOT/tools/cmd_sender/target/debug/cmd_sender"

kill_sim() {
    pkill -9 -f "zephyr.ex[e]" 2>/dev/null || true
    pkill -9 -f "g[z] sim" 2>/dev/null || true
    pkill -9 -f "gz-sim-mai[n]" 2>/dev/null || true
    pkill -9 -f "server/target/debug/serve[r]" 2>/dev/null || true
    pkill -9 -f "Xvf[b]" 2>/dev/null || true
    pkill -9 -f "slam_benchmark.py record-g[t]" 2>/dev/null || true
    sleep 1
}

# Blocks until the pose trace gains rows beyond its current size. Rows are
# written once per processed sweep by a LIVE robot session, so this is the
# one readiness signal that covers fresh start, session-up AND capabilities
# (sent on connect, before any sweep) — unlike grepping server.log, which
# can match leftovers from a previous run.
wait_for_trace_growth() {
    local before
    before=$(wc -l < "$HR_POSE_LOG" 2>/dev/null || echo 0)
    for _ in $(seq 1 60); do
        sleep 2
        local now
        now=$(wc -l < "$HR_POSE_LOG" 2>/dev/null || echo 0)
        [ "$now" -gt "$((before + 5))" ] && return 0
    done
    echo "FATAL: pose trace is not growing (no live robot session?)" >&2
    return 1
}

run_one() {
    local pattern=$1 run=$2
    local out="logs/bench/${pattern}-${run}"
    echo "=== baseline ${pattern} ${run} ==="
    kill_sim
    rm -rf "$out" house_map.bin
    rm -f logs/sim/server.log logs/sim/zephyr.log logs/sim/gazebo.log
    mkdir -p "$out"
    export HR_POSE_LOG="$ROOT/$out/trace.csv"

    if [ "$pattern" = "p4" ]; then
        export HR_FAULT_RIGHT_ENCODER=1
    else
        unset HR_FAULT_RIGHT_ENCODER || true
    fi

    ./tools/start_sim.sh --headless > "$out/sim.log" 2>&1 &
    wait_for_trace_growth
    sleep 10 # settle: gyro calibration + enough initial map for first plans

    if [ "$pattern" = "p5" ]; then
        python3 tools/slam_benchmark.py record-gt --out "$out/gt.csv" &
        local gt_pid=$!
        python3 tools/slam_benchmark.py drive --pattern p5a
        # Autosave interval is 30s; make sure the map+pose hit disk before the kill.
        sleep 40
        echo "--- killing server mid-run ---"
        pkill -9 -f "server/target/debug/serve[r]"
        sleep 2
        HEADLESS=1 HR_BIND=127.0.0.1 stdbuf -oL -eL "$SERVER_BIN" >> logs/sim/server.log 2>&1 &
        wait_for_trace_growth
        python3 tools/slam_benchmark.py drive --pattern p5b
        kill "$gt_pid" 2>/dev/null || true
        sleep 1
        python3 tools/slam_benchmark.py analyze --trace "$HR_POSE_LOG" --gt "$out/gt.csv" \
            --map house_map.bin --json "$out/metrics.json" --pattern p5
    else
        python3 tools/slam_benchmark.py run --pattern "$pattern" \
            --trace "$HR_POSE_LOG" --map house_map.bin --out-dir "$out"
    fi
    kill_sim
}

if [ "${1:-}" = "all" ]; then
    for pattern in p1 p2 p3 p4 p5; do
        for run in r1 r2 r3; do
            run_one "$pattern" "$run" || echo "RUN FAILED: $pattern $run (continuing)"
        done
    done
    python3 tools/slam_benchmark.py summarize logs/bench/*/metrics.json
else
    run_one "${1:?usage: slam_baseline.sh <p1|p2|p3|p4|p5|all> [run-id]}" "${2:-r1}"
fi
