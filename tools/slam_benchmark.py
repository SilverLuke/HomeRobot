#!/usr/bin/env python3
"""SLAM benchmark: trajectory + map quality metrics against sim ground truth.

Part of the ICP/SLAM rework (docs/specs/icp-slam-rework-plan.md, T2/T3).

Inputs:
  - Server pose trace CSV written with HR_POSE_LOG (one row per sweep):
      t_unix,odom_x,odom_y,odom_theta,slam_x,slam_y,slam_theta,score,mode
  - Ground-truth CSV (recorded from Gazebo, see `record-gt` [T2b]):
      t_unix,x,y,theta
  - Optionally the persisted map (house_map.bin, HRWORLD1) for map metrics.

Metrics (design doc §5.3): ATE RMSE, RPE (1m segments, translation m/m and
rotation deg/m), revisit error at the final sample, wall-hit fidelity and
free-beyond-walls count.

Usage:
  slam_benchmark.py self-test
  slam_benchmark.py analyze --trace pose.csv --gt gt.csv \
      [--map house_map.bin] [--json out.json]

Pure stdlib on purpose: must run inside the build machine's nix-shell
without extra Python packages.
"""

import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def wrap_angle(a):
    """Normalize an angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def kabsch_2d(est_xy, gt_xy):
    """Optimal rigid transform (theta, tx, ty) mapping est onto gt.

    Minimizes sum |R(theta) * e + t - g|^2 (2D Procrustes, no scale).
    """
    n = len(est_xy)
    if n == 0:
        return 0.0, 0.0, 0.0
    me_x = sum(p[0] for p in est_xy) / n
    me_y = sum(p[1] for p in est_xy) / n
    mg_x = sum(p[0] for p in gt_xy) / n
    mg_y = sum(p[1] for p in gt_xy) / n

    sxx = 0.0  # cos accumulator
    sxy = 0.0  # sin accumulator
    for (ex, ey), (gx, gy) in zip(est_xy, gt_xy):
        ex -= me_x
        ey -= me_y
        gx -= mg_x
        gy -= mg_y
        sxx += gx * ex + gy * ey
        sxy += gy * ex - gx * ey
    theta = math.atan2(sxy, sxx)
    c, s = math.cos(theta), math.sin(theta)
    tx = mg_x - (me_x * c - me_y * s)
    ty = mg_y - (me_x * s + me_y * c)
    return theta, tx, ty


def apply_transform(transform, x, y, heading=None):
    theta, tx, ty = transform
    c, s = math.cos(theta), math.sin(theta)
    xr = x * c - y * s + tx
    yr = x * s + y * c + ty
    if heading is None:
        return xr, yr
    return xr, yr, wrap_angle(heading + theta)


def relative_se2(a, b):
    """Pose of b expressed in a's frame; poses are (x, y, theta)."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap_angle(b[2] - a[2]),
    )


# ---------------------------------------------------------------------------
# Input loading and pairing
# ---------------------------------------------------------------------------

TRACE_HEADER = "t_unix,odom_x,odom_y,odom_theta,slam_x,slam_y,slam_theta,score,mode"


def load_pose_trace(path):
    """Returns (slam_poses, odom_poses) as lists of (t, x, y, theta)."""
    slam, odom = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("t_unix"):
                continue
            cols = line.split(",")
            t = float(cols[0])
            odom.append((t, float(cols[1]), float(cols[2]), float(cols[3])))
            slam.append((t, float(cols[4]), float(cols[5]), float(cols[6])))
    return slam, odom


def load_gt(path):
    """Ground-truth CSV rows (t, x, y, theta)."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("t_unix"):
                continue
            cols = line.split(",")
            out.append((float(cols[0]), float(cols[1]), float(cols[2]), float(cols[3])))
    return out


def pair_trajectories(est, gt, max_dt=0.15):
    """Nearest-in-time pairing of two (t, x, y, theta) series.

    Returns parallel lists of (x, y, theta). Both inputs must be time-sorted.
    """
    pairs_est, pairs_gt = [], []
    j = 0
    for t, x, y, th in est:
        while j + 1 < len(gt) and abs(gt[j + 1][0] - t) <= abs(gt[j][0] - t):
            j += 1
        if not gt or abs(gt[j][0] - t) > max_dt:
            continue
        pairs_est.append((x, y, th))
        pairs_gt.append((gt[j][1], gt[j][2], gt[j][3]))
    return pairs_est, pairs_gt


# ---------------------------------------------------------------------------
# Trajectory metrics
# ---------------------------------------------------------------------------

def ate_rmse(est, gt, transform):
    """RMSE of positional error after applying `transform` to est."""
    if not est:
        return float("nan")
    total = 0.0
    for e, g in zip(est, gt):
        x, y = apply_transform(transform, e[0], e[1])
        total += (x - g[0]) ** 2 + (y - g[1]) ** 2
    return math.sqrt(total / len(est))


def arc_lengths(poses):
    cum = [0.0]
    for a, b in zip(poses, poses[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return cum


def rpe(est, gt, segment_m=1.0):
    """Mean relative pose error over `segment_m` GT arc-length segments.

    Returns (translation m/m, rotation deg/m, segment count). Invariant to
    the global alignment, so `est` is used un-transformed.
    """
    cum = arc_lengths(gt)
    errs_t, errs_r = [], []
    j = 0
    for i in range(len(gt)):
        while j < len(gt) and cum[j] - cum[i] < segment_m:
            j += 1
        if j >= len(gt):
            break
        seg = cum[j] - cum[i]
        rel_gt = relative_se2(gt[i], gt[j])
        rel_est = relative_se2(est[i], est[j])
        errs_t.append(math.hypot(rel_est[0] - rel_gt[0], rel_est[1] - rel_gt[1]) / seg)
        errs_r.append(abs(wrap_angle(rel_est[2] - rel_gt[2])) / seg)
    if not errs_t:
        return float("nan"), float("nan"), 0
    mean_t = sum(errs_t) / len(errs_t)
    mean_r_deg = math.degrees(sum(errs_r) / len(errs_r))
    return mean_t, mean_r_deg, len(errs_t)


def revisit_error(est, gt, transform):
    """Positional and heading error (m, deg) at the final paired sample."""
    if not est:
        return float("nan"), float("nan")
    e, g = est[-1], gt[-1]
    x, y, th = apply_transform(transform, e[0], e[1], e[2])
    return math.hypot(x - g[0], y - g[1]), abs(math.degrees(wrap_angle(th - g[2])))


# ---------------------------------------------------------------------------
# Map metrics (HRWORLD1)
# ---------------------------------------------------------------------------

CELL_FREE = -20
CELL_OCCUPIED = 20


def load_world_map(path):
    """Parse house_map.bin (magic HRWORLD1, see server/src/world.rs)."""
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:8] != b"HRWORLD1":
        raise ValueError("bad magic in %s" % path)
    width, height = struct.unpack_from("<II", raw, 8)
    resolution, origin_x, origin_y = struct.unpack_from("<fff", raw, 16)
    px, py, ptheta = struct.unpack_from("<fff", raw, 28)
    (initialized,) = struct.unpack_from("<B", raw, 40)
    data = struct.unpack_from("<%dh" % (width * height), raw, 41)
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "pose": (px, py, ptheta),
        "initialized": bool(initialized),
        "data": data,
    }


def cell_to_world(grid, gx, gy):
    """Grid indices to SLAM-frame meters (matches mapping.rs)."""
    x = (gx - grid["width"] / 2.0) * grid["resolution"] + grid["origin_x"]
    y = (gy - grid["height"] / 2.0) * grid["resolution"] + grid["origin_y"]
    return x, y


class Arena:
    """Structure model of simulation/sim.world for map-fidelity metrics.

    Outer square (wall centerlines at +-half), plus interior obstacles:
    a rotated wall segment, an axis-aligned box, and a cylinder.
    """

    def __init__(self, half=5.0):
        self.half = half
        # obs1: 2m wall centered (2,2), yaw 0.785
        c, s = math.cos(0.785), math.sin(0.785)
        self.seg_a = (2.0 - c, 2.0 - s)
        self.seg_b = (2.0 + c, 2.0 + s)
        # obs2: 0.5 x 3.0 box centered (-2,-2)
        self.box_center = (-2.0, -2.0)
        self.box_half = (0.25, 1.5)
        # obs3: cylinder r=0.5 at origin
        self.cyl_r = 0.5

    def _seg_dist(self, x, y):
        ax, ay = self.seg_a
        bx, by = self.seg_b
        dx, dy = bx - ax, by - ay
        t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return math.hypot(x - (ax + t * dx), y - (ay + t * dy))

    def _box_dist(self, x, y):
        dx = max(abs(x - self.box_center[0]) - self.box_half[0], 0.0)
        dy = max(abs(y - self.box_center[1]) - self.box_half[1], 0.0)
        return math.hypot(dx, dy)

    def structure_dist(self, x, y):
        """Distance to the nearest wall/obstacle surface (approx centerlines)."""
        outer = abs(max(abs(x), abs(y)) - self.half)
        circle = abs(math.hypot(x, y) - self.cyl_r)
        return min(outer, self._seg_dist(x, y), self._box_dist(x, y), circle)

    def outside(self, x, y, margin=0.05):
        return max(abs(x), abs(y)) > self.half + margin


def map_metrics(grid, transform, arena, wall_tol=0.10):
    """(wall_hit_pct, occupied_count, free_beyond_walls_count).

    `transform` maps SLAM frame to the GT world frame (from trajectory
    alignment); interior structures are part of the fidelity check.
    """
    occupied = 0
    occupied_on_wall = 0
    free_beyond = 0
    w = grid["width"]
    for idx, val in enumerate(grid["data"]):
        if val < CELL_FREE or val >= CELL_OCCUPIED:
            gx, gy = idx % w, idx // w
            x, y = apply_transform(transform, *cell_to_world(grid, gx, gy))
            if val >= CELL_OCCUPIED:
                occupied += 1
                if arena.structure_dist(x, y) <= wall_tol:
                    occupied_on_wall += 1
            elif arena.outside(x, y):
                free_beyond += 1
    pct = (100.0 * occupied_on_wall / occupied) if occupied else float("nan")
    return pct, occupied, free_beyond


# ---------------------------------------------------------------------------
# Ground-truth recording (Gazebo, build machine only)
# ---------------------------------------------------------------------------

POSE_TOPIC = "/world/homerobot_world/pose/info"
_NUM = r":\s*([-0-9.eE+]+)"


def parse_pose_block(buf, model="homerobot"):
    """Extract the model's pose from accumulated `gz topic -e` text output.

    Returns ((x, y, theta), consumed_index) when a complete block is
    available, None while it is still partial. Same text-format parsing as
    tools/regression_test.py, adapted for a streaming buffer.
    """
    start = buf.find('name: "%s"' % model)
    if start == -1:
        return None
    window = buf[start:start + 1200]
    pos = re.search(r"position\s*\{([^}]*)\}", window)
    orient = re.search(r"orientation\s*\{([^}]*)\}", window)
    if not pos or not orient:
        return None

    def field(name, text, default=0.0):
        m = re.search(name + _NUM, text)
        return float(m.group(1)) if m else default

    x = field("x", pos.group(1))
    y = field("y", pos.group(1))
    oz = field("z", orient.group(1))
    ow = field("w", orient.group(1), 1.0)
    theta = math.atan2(2.0 * ow * oz, 1.0 - 2.0 * oz * oz)
    return (x, y, theta), start + orient.end()


class GTMonitor(threading.Thread):
    """Streams Gazebo ground truth in the background.

    Always keeps `latest` = (t_unix, x, y, theta) for liveness/leg checks;
    additionally appends throttled CSV rows when `out_path` is given.
    """

    def __init__(self, out_path=None, hz=20.0):
        super().__init__(daemon=True)
        self.out_path = out_path
        self.min_interval = 1.0 / hz
        self.latest = None
        self.rows = 0
        self._stop = threading.Event()
        self._proc = None

    def run(self):
        env = os.environ.copy()
        env.setdefault("GZ_IP", "127.0.0.1")
        env.setdefault("GZ_PARTITION", "homerobot_sim")
        cmd = ["gz", "topic", "-t", POSE_TOPIC, "-e"]
        # Force line buffering: a block-buffered pipe would make `latest` lag
        # by seconds, breaking both leg settling and time pairing.
        if os.path.exists("/usr/bin/stdbuf") or os.system("command -v stdbuf >/dev/null 2>&1") == 0:
            cmd = ["stdbuf", "-oL"] + cmd
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env,
        )
        out = open(self.out_path, "w") if self.out_path else None
        if out:
            out.write("t_unix,x,y,theta\n")
        last_row = 0.0
        buf = ""
        try:
            for line in self._proc.stdout:
                if self._stop.is_set():
                    break
                buf += line
                parsed = parse_pose_block(buf)
                if parsed is None:
                    if len(buf) > 65536:
                        buf = buf[-4096:]
                    continue
                (x, y, theta), consumed = parsed
                buf = buf[consumed:]
                now = time.time()
                self.latest = (now, x, y, theta)
                if out and now - last_row >= self.min_interval:
                    out.write("%.6f,%.6f,%.6f,%.6f\n" % (now, x, y, theta))
                    out.flush()
                    last_row = now
                    self.rows += 1
        finally:
            if out:
                out.close()

    def stop(self):
        self._stop.set()
        if self._proc:
            self._proc.terminate()

    def wait_alive(self, timeout=10.0):
        """True once the first sample arrives (i.e. the sim is up)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.latest is not None:
                return True
            time.sleep(0.2)
        return False


def record_gt(out_path, duration=0.0, hz=20.0):
    """Stream ground truth to CSV until `duration` elapses (0 = until ^C)."""
    monitor = GTMonitor(out_path, hz)
    monitor.start()
    try:
        if duration:
            time.sleep(duration)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()
        monitor.join(timeout=3.0)
    print("recorded %d ground-truth samples to %s" % (monitor.rows, out_path))
    if monitor.rows == 0:
        print("no samples — is the simulation running (GZ_PARTITION=homerobot_sim)?")
        return 1
    return 0


# ---------------------------------------------------------------------------
# Drive patterns (design doc §5.2). Waypoints are in the SERVER's SLAM frame,
# which is zeroed at the spawn pose: world (1, 0, 0) => slam (0, 0, 0).
# ---------------------------------------------------------------------------

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# P1 out-and-back: closed-loop firmware motions, SLAM-independent.
# P2 perimeter loop: goto circuit hugging the arena interior; legs keep
#     >=0.7m clearance from the interior obstacles and outer walls.
# P3 room-hop: goto legs threading between the cylinder, the diagonal wall
#     and the box, ending back at the start.
# P4 = P1's motion sequence with the server started under
#     HR_FAULT_RIGHT_ENCODER=1: firmware closed-loop motions run on the
#     robot's own (clean) encoders, so the physical tour is guaranteed and
#     P4-vs-P1 is a controlled fault/nominal comparison on identical motion.
#     (A goto tour under the fault measured nothing: BasicSlam navigation
#     cannot even start, GT path length was 0.0 across all runs.)
# P5 restart: p5a drives out, the harness (tools/slam_baseline.sh) kills and
#     restarts the server, p5b drives back — one GT recording spans both.
P1_MOTIONS = [
    ("motion", ["--action", "straight", "--distance", "2.0"]),
    ("motion", ["--action", "rotate", "--angle", "180"]),
    ("motion", ["--action", "straight", "--distance", "2.0"]),
    ("motion", ["--action", "rotate", "--angle", "180"]),
]

PATTERNS = {
    "p1": P1_MOTIONS,
    "p2": [
        ("goto", (2.5, 3.5)),
        ("goto", (-4.5, 3.5)),
        ("goto", (-4.5, -4.2)),
        ("goto", (2.5, -4.2)),
        ("goto", (0.0, 0.0)),
    ],
    "p3": [
        ("goto", (-1.0, 2.8)),
        ("goto", (-4.0, 0.0)),
        ("goto", (-0.5, -2.8)),
        ("goto", (0.0, 0.0)),
    ],
    "p4": P1_MOTIONS,
    "p5a": [
        ("motion", ["--action", "straight", "--distance", "2.0"]),
    ],
    "p5b": [
        ("motion", ["--action", "rotate", "--angle", "180"]),
        ("motion", ["--action", "straight", "--distance", "2.0"]),
        ("motion", ["--action", "rotate", "--angle", "180"]),
    ],
}


def run_cmd_sender(host, args, timeout):
    """Invoke tools/cmd_sender in proxy mode; HR_CMD_SENDER can point at a
    prebuilt binary, otherwise cargo-run from its crate directory."""
    exe = os.environ.get("HR_CMD_SENDER")
    if exe:
        cmd, cwd = [exe], None
    else:
        cmd, cwd = ["cargo", "run", "--quiet", "--"], os.path.join(REPO_ROOT, "tools", "cmd_sender")
    cmd += ["--proxy", "--host", host] + args
    print("  $ cmd_sender %s" % " ".join(args))
    try:
        subprocess.run(cmd, cwd=cwd, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        print("  (cmd_sender timed out after %ds)" % timeout)


def wait_until_stationary(get_latest, timeout, settle=3.0, min_move=0.05,
                          grace=4.0, poll=0.5, sleep_fn=time.sleep, now_fn=time.time):
    """Wait until ground truth stops moving (goal reached or nav gave up).

    True when the robot moved less than `min_move` over the last `settle`
    seconds (after an initial `grace` period), False on timeout. Clock and
    sleep are injectable for the self-test.
    """
    history = []
    start = now_fn()
    while now_fn() - start < timeout:
        latest = get_latest()
        if latest is not None:
            now = now_fn()
            history.append((now, latest[1], latest[2]))
            history = [h for h in history if now - h[0] <= settle + poll]
            if now - start >= grace and len(history) >= 3 and now - history[0][0] >= settle:
                xs = [h[1] for h in history]
                ys = [h[2] for h in history]
                if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < min_move:
                    return True
        sleep_fn(poll)
    return False


def drive_pattern(pattern, host, monitor, leg_timeout=120.0):
    """Execute one named pattern; leg completion is judged on ground truth."""
    for step, arg in PATTERNS[pattern]:
        if step == "motion":
            # ExecuteMotion blocks in cmd_sender until the firmware reports
            # completion; the stationary check is just a safety net.
            run_cmd_sender(host, ["motion"] + arg, timeout=90)
            wait_until_stationary(lambda: monitor.latest, timeout=15.0, grace=0.0)
        elif step == "goto":
            x, y = arg
            # Navigation can abort instantly from a cold start (sparse map =>
            # A* fails => NavigateTo falls back to Manual). Retry the leg
            # until ground truth shows actual displacement.
            start_pos = monitor.latest
            for attempt in range(3):
                run_cmd_sender(host, ["go-to", str(x), str(y)], timeout=30)
                done = wait_until_stationary(lambda: monitor.latest, timeout=leg_timeout)
                cur = monitor.latest
                moved = (
                    start_pos is not None and cur is not None
                    and math.hypot(cur[1] - start_pos[1], cur[2] - start_pos[2]) >= 0.2
                )
                print("  leg (%.1f, %.1f) attempt %d: %s%s"
                      % (x, y, attempt + 1, "settled" if done else "TIMEOUT",
                         "" if moved else ", no displacement"))
                if moved:
                    break
    run_cmd_sender(host, ["stop"], timeout=30)


# ---------------------------------------------------------------------------
# Analysis entry point
# ---------------------------------------------------------------------------

def analyze(trace_path, gt_path, map_path=None, arena_half=5.0):
    slam, odom = load_pose_trace(trace_path)
    gt = load_gt(gt_path)
    est_p, gt_p = pair_trajectories(slam, gt)
    if len(est_p) < 10:
        raise SystemExit(
            "only %d paired samples between %s and %s — clocks aligned?"
            % (len(est_p), trace_path, gt_path)
        )

    transform = kabsch_2d(
        [(p[0], p[1]) for p in est_p], [(p[0], p[1]) for p in gt_p]
    )
    ate = ate_rmse(est_p, gt_p, transform)
    rpe_t, rpe_r, segments = rpe(est_p, gt_p)
    rev_t, rev_r = revisit_error(est_p, gt_p, transform)

    # Odometry-only trajectory scored the same way, as the "no SLAM" floor.
    odom_p, gt_odom_p = pair_trajectories(odom, gt)
    odom_ate = ate_rmse(
        odom_p, gt_odom_p,
        kabsch_2d([(p[0], p[1]) for p in odom_p], [(p[0], p[1]) for p in gt_odom_p]),
    )

    result = {
        "paired_samples": len(est_p),
        "gt_path_length_m": round(arc_lengths(gt_p)[-1], 3),
        "ate_rmse_m": round(ate, 4),
        "odom_ate_rmse_m": round(odom_ate, 4),
        "rpe_trans_m_per_m": round(rpe_t, 4),
        "rpe_rot_deg_per_m": round(rpe_r, 4),
        "rpe_segments": segments,
        "revisit_trans_m": round(rev_t, 4),
        "revisit_rot_deg": round(rev_r, 4),
    }
    if map_path:
        grid = load_world_map(map_path)
        pct, occ, beyond = map_metrics(grid, transform, Arena(arena_half))
        result["wall_hit_pct"] = round(pct, 2)
        result["occupied_cells"] = occ
        result["free_beyond_walls"] = beyond
    return result


# ---------------------------------------------------------------------------
# Summarize: aggregate metrics.json files into a markdown baseline table
# ---------------------------------------------------------------------------

SUMMARY_KEYS = [
    ("ate_rmse_m", "ATE (m)"),
    ("odom_ate_rmse_m", "odom ATE (m)"),
    ("rpe_trans_m_per_m", "RPE t (m/m)"),
    ("rpe_rot_deg_per_m", "RPE r (deg/m)"),
    ("revisit_trans_m", "revisit (m)"),
    ("revisit_rot_deg", "revisit (deg)"),
    ("gt_path_length_m", "path (m)"),
    ("wall_hit_pct", "wall-hit %"),
    ("free_beyond_walls", "free-beyond"),
]


def summarize(paths):
    """Group metrics.json files by pattern; print mean (min..max) per metric."""
    by_pattern = {}
    for path in paths:
        with open(path) as f:
            m = json.load(f)
        by_pattern.setdefault(m.get("pattern", "?"), []).append(m)

    header = ["pattern", "runs"] + [label for _, label in SUMMARY_KEYS]
    print("| " + " | ".join(header) + " |")
    print("|" + "---|" * len(header))
    for pattern in sorted(by_pattern):
        runs = by_pattern[pattern]
        cells = [pattern, str(len(runs))]
        for key, _ in SUMMARY_KEYS:
            vals = [r[key] for r in runs if key in r and not math.isnan(r[key])]
            if not vals:
                cells.append("—")
            elif len(vals) == 1:
                cells.append("%.3f" % vals[0])
            else:
                cells.append("%.3f (%.3f..%.3f)"
                             % (sum(vals) / len(vals), min(vals), max(vals)))
        print("| " + " | ".join(cells) + " |")


# ---------------------------------------------------------------------------
# Self-test (pure python, no Gazebo): synthetic trajectories with known
# injected errors must yield the expected metric values.
# ---------------------------------------------------------------------------

def _circle_traj(n=300, radius=2.0, hz=5.0):
    out = []
    for i in range(n):
        phi = 2.0 * math.pi * i / n
        out.append((
            i / hz,
            radius * math.cos(phi),
            radius * math.sin(phi),
            wrap_angle(phi + math.pi / 2.0),
        ))
    return out


def self_test():
    failures = []

    def check(name, value, expected, tol):
        ok = abs(value - expected) <= tol
        print("  [%s] %-42s %.5f (expected %.5f ±%.5f)"
              % ("PASS" if ok else "FAIL", name, value, expected, tol))
        if not ok:
            failures.append(name)

    gt = _circle_traj()

    # 1. Rigidly transformed copy must align to ~zero ATE.
    rot, tx, ty = 0.7, 1.5, -2.0
    moved = [(t,) + apply_transform((rot, tx, ty), x, y, th) for t, x, y, th in gt]
    e, g = pair_trajectories(moved, gt)
    tr = kabsch_2d([(p[0], p[1]) for p in e], [(p[0], p[1]) for p in g])
    check("kabsch recovers rigid transform (ATE)", ate_rmse(e, g, tr), 0.0, 1e-6)

    # 2. Alternating perpendicular offsets of 5cm -> ATE RMSE = 5cm.
    d = 0.05
    noisy = []
    for i, (t, x, y, th) in enumerate(gt):
        sign = 1.0 if i % 2 == 0 else -1.0
        noisy.append((t, x + sign * d * math.cos(th + math.pi / 2),
                      y + sign * d * math.sin(th + math.pi / 2), th))
    e, g = pair_trajectories(noisy, gt)
    tr = kabsch_2d([(p[0], p[1]) for p in e], [(p[0], p[1]) for p in g])
    check("ATE of ±5cm offsets", ate_rmse(e, g, tr), d, 0.002)

    # 3. 2% scale drift -> RPE translation ~0.02 m/m.
    scaled = [(t, 1.02 * x, 1.02 * y, th) for t, x, y, th in gt]
    e, g = pair_trajectories(scaled, gt)
    rpe_t, _, nseg = rpe(e, g)
    check("RPE translation of 2pct scale", rpe_t, 0.02, 0.004)
    if nseg < 10:
        failures.append("rpe produced too few segments (%d)" % nseg)

    # 4. Heading drift of 0.5 deg per meter -> RPE rotation ~0.5 deg/m.
    drift = math.radians(0.5)
    cum = arc_lengths([(x, y) for _, x, y, _ in gt])
    drifty = [(t, x, y, wrap_angle(th + drift * cum[i]))
              for i, (t, x, y, th) in enumerate(gt)]
    e, g = pair_trajectories(drifty, gt)
    _, rpe_r, _ = rpe(e, g)
    check("RPE rotation of 0.5deg/m drift", rpe_r, 0.5, 0.05)

    # 5. Linearly growing offset ending at 10cm -> revisit error ~10cm.
    # Kabsch will absorb part of a linear drift; inject it only over the
    # final quarter so the alignment stays pinned by the clean majority.
    n = len(gt)
    end_err = 0.10
    drifted = []
    for i, (t, x, y, th) in enumerate(gt):
        frac = max(0.0, (i - 0.75 * n) / (0.25 * n))
        drifted.append((t, x + end_err * frac, y, th))
    e, g = pair_trajectories(drifted, gt)
    tr = kabsch_2d([(p[0], p[1]) for p in e], [(p[0], p[1]) for p in g])
    rev_t, _ = revisit_error(e, g, tr)
    check("revisit error of 10cm end drift", rev_t, end_err, 0.03)

    # 6. Time pairing: 40ms skew pairs everything; samples outside the GT
    # time coverage are dropped (a constant skew within coverage only shifts
    # correspondence — nearest-neighbor still pairs, by design).
    skewed = [(t + 0.04, x, y, th) for t, x, y, th in gt]
    e, _ = pair_trajectories(skewed, gt)
    check("pairing tolerates 40ms skew (fraction)", len(e) / len(gt), 1.0, 0.01)
    e, _ = pair_trajectories(gt, gt[: len(gt) // 2])
    check("pairing drops samples beyond GT coverage", len(e) / len(gt), 0.5, 0.02)

    # 7. HRWORLD1 round-trip + map metrics on a synthetic arena square.
    import tempfile
    w = h = 240
    res = 0.05
    data = [0] * (w * h)
    half_cells = int(5.0 / res)
    for i in range(-half_cells, half_cells + 1):
        for (gx, gy) in [
            (w // 2 + i, h // 2 - half_cells), (w // 2 + i, h // 2 + half_cells),
            (w // 2 - half_cells, h // 2 + i), (w // 2 + half_cells, h // 2 + i),
        ]:
            data[gy * w + gx] = 50  # occupied on the arena wall centerline
    data[(h // 2) * w + (w // 2 + half_cells + 20)] = -40  # free 1m beyond wall
    blob = b"HRWORLD1" + struct.pack("<II", w, h) + struct.pack("<fff", res, 0.0, 0.0)
    blob += struct.pack("<fffB", 0.0, 0.0, 0.0, 1)
    blob += struct.pack("<%dh" % (w * h), *data)
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(blob)
        path = f.name
    grid = load_world_map(path)
    pct, occ, beyond = map_metrics(grid, (0.0, 0.0, 0.0), Arena(5.0))
    check("map wall-hit pct on perfect walls", pct, 100.0, 0.01)
    check("free-beyond-walls detects the stray cell", float(beyond), 1.0, 0.01)
    # Square boundary of side 2*half_cells+1: 4 corners shared between walls.
    expected_occ = 8 * half_cells
    if occ != expected_occ:
        failures.append("occupied count %d != %d" % (occ, expected_occ))

    # 8. gz pose_v text parsing: 90deg yaw quaternion, split across "lines"
    # like the streaming reader sees it.
    sample = (
        'pose {\n  name: "other_model"\n  position { x: 9 y: 9 z: 0 }\n'
        '  orientation { x: 0 y: 0 z: 0 w: 1 }\n}\n'
        'pose {\n  name: "homerobot"\n'
        "  position {\n    x: 1.5\n    y: -0.5\n    z: 0.1\n  }\n"
        "  orientation {\n    x: 0\n    y: 0\n    z: 0.7071068\n    w: 0.7071068\n  }\n}\n"
    )
    parsed = parse_pose_block(sample)
    if parsed is None:
        failures.append("pose_v parser returned None")
    else:
        (x, y, theta), consumed = parsed
        check("pose_v parser x", x, 1.5, 1e-6)
        check("pose_v parser y", y, -0.5, 1e-6)
        check("pose_v parser theta (90deg yaw)", theta, math.pi / 2.0, 1e-4)
        if consumed <= 0 or consumed > len(sample):
            failures.append("pose_v parser consumed index out of range")
    if parse_pose_block('pose {\n  name: "homerobot"\n  position { x: 1') is not None:
        failures.append("pose_v parser must wait for a complete block")

    # 9. Stationary detector with an injected clock: a robot that drives for
    # 10s then stops settles shortly after; one that never stops times out.
    def fake_run(stop_at):
        clock = [0.0]

        def now():
            return clock[0]

        def sleep(dt):
            clock[0] += dt

        def latest():
            t = clock[0]
            x = 0.2 * min(t, stop_at)
            return (t, x, 0.0, 0.0)

        done = wait_until_stationary(latest, timeout=60.0,
                                     sleep_fn=sleep, now_fn=now)
        return done, clock[0]

    done, elapsed = fake_run(stop_at=10.0)
    if not done:
        failures.append("stationary detector missed the stop")
    check("stationary detector settle time", elapsed, 13.5, 2.0)
    done, elapsed = fake_run(stop_at=1e9)
    if done:
        failures.append("stationary detector fired on a moving robot")
    check("stationary detector timeout", elapsed, 60.0, 1.0)

    print()
    if failures:
        print("SELF-TEST FAILED: %s" % ", ".join(failures))
        return 1
    print("SELF-TEST PASSED (%d checks)" % 14)
    return 0


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("self-test", help="run metric unit checks (no Gazebo)")

    p_an = sub.add_parser("analyze", help="compute metrics from recorded files")
    p_an.add_argument("--trace", required=True, help="HR_POSE_LOG CSV")
    p_an.add_argument("--gt", required=True, help="ground-truth CSV")
    p_an.add_argument("--map", help="house_map.bin for map metrics")
    p_an.add_argument("--arena-half", type=float, default=5.0)
    p_an.add_argument("--json", help="write metrics JSON here")
    p_an.add_argument("--pattern", help="pattern label recorded in the JSON")

    p_rec = sub.add_parser("record-gt", help="stream Gazebo ground truth to CSV")
    p_rec.add_argument("--out", required=True)
    p_rec.add_argument("--duration", type=float, default=0.0,
                       help="seconds to record (0 = until interrupted)")
    p_rec.add_argument("--hz", type=float, default=20.0)

    p_dr = sub.add_parser("drive", help="execute a drive pattern (sim must be up)")
    p_dr.add_argument("--pattern", required=True, choices=sorted(PATTERNS))
    p_dr.add_argument("--host", default="127.0.0.1")

    p_run = sub.add_parser("run", help="record + drive + analyze in one command")
    p_run.add_argument("--pattern", required=True, choices=sorted(PATTERNS))
    p_run.add_argument("--trace", required=True,
                       help="pose CSV the server writes (start it with HR_POSE_LOG)")
    p_run.add_argument("--map", help="house_map.bin for map metrics")
    p_run.add_argument("--out-dir", required=True)
    p_run.add_argument("--host", default="127.0.0.1")
    p_run.add_argument("--arena-half", type=float, default=5.0)

    p_sum = sub.add_parser("summarize", help="aggregate metrics.json files into a table")
    p_sum.add_argument("files", nargs="+")

    args = parser.parse_args()
    if args.cmd == "summarize":
        summarize(args.files)
        sys.exit(0)
    if args.cmd == "self-test":
        sys.exit(self_test())
    if args.cmd == "record-gt":
        sys.exit(record_gt(args.out, args.duration, args.hz))
    if args.cmd == "drive":
        monitor = GTMonitor()
        monitor.start()
        if not monitor.wait_alive():
            sys.exit("no ground truth — is the simulation running?")
        drive_pattern(args.pattern, args.host, monitor)
        monitor.stop()
        sys.exit(0)
    if args.cmd == "run":
        os.makedirs(args.out_dir, exist_ok=True)
        gt_path = os.path.join(args.out_dir, "gt.csv")
        monitor = GTMonitor(out_path=gt_path)
        monitor.start()
        if not monitor.wait_alive():
            sys.exit("no ground truth — is the simulation running?")
        drive_pattern(args.pattern, args.host, monitor)
        time.sleep(2.0)  # let the final sweeps land in the trace
        monitor.stop()
        monitor.join(timeout=3.0)
        print("recorded %d ground-truth samples" % monitor.rows)
        result = analyze(args.trace, gt_path, args.map, args.arena_half)
        result["pattern"] = args.pattern
        for key, value in result.items():
            print("%-24s %s" % (key, value))
        json_path = os.path.join(args.out_dir, "metrics.json")
        with open(json_path, "w") as f:
            json.dump(result, f, indent=2)
        print("metrics written to %s" % json_path)
        sys.exit(0)
    if args.cmd == "analyze":
        result = analyze(args.trace, args.gt, args.map, args.arena_half)
        if args.pattern:
            result["pattern"] = args.pattern
        for key, value in result.items():
            print("%-24s %s" % (key, value))
        if args.json:
            with open(args.json, "w") as f:
                json.dump(result, f, indent=2)
        sys.exit(0)


if __name__ == "__main__":
    main()
