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
import struct
import sys

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

    print()
    if failures:
        print("SELF-TEST FAILED: %s" % ", ".join(failures))
        return 1
    print("SELF-TEST PASSED (%d checks)" % 9)
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

    args = parser.parse_args()
    if args.cmd == "self-test":
        sys.exit(self_test())
    if args.cmd == "analyze":
        result = analyze(args.trace, args.gt, args.map, args.arena_half)
        for key, value in result.items():
            print("%-24s %s" % (key, value))
        if args.json:
            with open(args.json, "w") as f:
                json.dump(result, f, indent=2)
        sys.exit(0)


if __name__ == "__main__":
    main()
