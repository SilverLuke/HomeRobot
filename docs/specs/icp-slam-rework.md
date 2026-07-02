# ICP/SLAM Quality Rework — Design

Status: **DRAFT — for review, no implementation yet.**
Source: `docs/specs/icp-slam-rework-prompt.md`, codebase review of 2026-07-02.

## 1. Problem statement

`BasicSlam` matches each sweep against a rolling window of the last 1000
points. Consequences (prompt §"Known weaknesses"):

1. No drift correction on revisit — the reference cloud forgets everything
   older than ~3 sweeps, so error grows without bound and painted free space
   beyond the arena walls in sim.
2. Brute-force nearest neighbor: ~4.5M distance checks per sweep.
3. The authoritative artifact — the log-odds occupancy grid — is written but
   never matched against.
4. No relocalization after restart (dead-reckoning until the cloud refills,
   kept alive by the `update_count = 1` hack) and no divergence detection.

## 2. Constraints that drive the design

These system properties decide the architecture more than algorithm taste
does:

- **Sparse, gappy scans.** ~360 points/rev at 5–6Hz, with recurring missing
  angular slices up to ~8.5° clustered in a fixed fan (issues.md #7).
  Point-to-point correspondence is weak with this density; any matcher that
  scores points *independently* against a smooth cost surface is structurally
  more robust here.
- **Good heading prior, currently-broken translation prior.** Heading is 98%
  gyro (`odometry.rs`) and trustworthy. Translation comes from encoders, and
  the right encoder is faulty hardware-side (issues.md #6: always-positive +
  phantom ticks at standstill). The matcher must tolerate a *garbage
  translation prior*, not merely a noisy one.
- **Small, bounded environment.** A home: the whole map is 30×30m @ 5cm and
  fits in one grid. Robot speed ≤ ~0.3 m/s → inter-sweep motion ≤ ~10cm and
  ≤ ~10° at 5Hz. A bounded search window around the prediction always
  contains the true pose.
- **The grid is already persisted** (`HRWORLD1`), the ICP cloud is not.
  A matcher whose reference *is* the grid gets restart-relocalization
  almost for free and needs no persistence format change.
- **Desktop CPU budget, debug-build tests must stay fast.** We have headroom,
  but tests must not embed Gazebo or big search loops.
- **No ROS, minimal deps.** Everything below is implementable with hand-rolled
  math; the only candidate dependency is `nalgebra` for 3×3 covariance work
  (see Open Questions).

## 3. Options compared

### (a) Scan-to-map matching against the log-odds grid

Match each (deskewed) sweep directly against a **likelihood field** derived
from the occupancy grid: a raster where each cell holds "how close is this
cell to known-occupied structure" (Gaussian-blurred occupancy / distance
transform). Two families:

- *Gradient-based* (Hector SLAM): Gauss–Newton on a bilinearly interpolated
  field. Fast, sub-cell accurate, but needs a good initial guess (basin
  ~±1–2 cells) — fragile exactly when odometry is bad.
- *Correlative* (Olson 2009): exhaustively score a discretized window of
  candidate poses (x, y, θ) at coarse resolution, refine the winner at fine
  resolution. No initial-guess fragility inside the window; returns a score
  surface from which match confidence/covariance falls out.

Pros: map-anchored (revisit error bounded by map consistency, no separate
loop-closure machinery for a home-sized environment); no correspondence
problem (each point scored independently — robust to gaps and sparsity);
reference is the persisted artifact → restart relocalization is the same code
with a wider window; brute-force NN disappears.
Cons: map-contamination feedback (a bad pose writes bad cells that future
matches lock onto) — must gate map updates on match quality; the field must be
kept in sync with the grid (rebuild cost).

### (b) kd-tree / grid-bucketed ICP against persistent keyframe clouds

Keep ICP but fix the two implementation sins: bucket/kd-tree correspondences
(O(N log M)) and a persistent keyframe store (submaps with poses) instead of
the rolling window.

Pros: proven pipeline; sub-resolution accuracy on dense scans; incremental
change from today's code.
Cons: still point-to-point on ~300 sparse points — correspondence quality is
the real weakness, not NN speed; drift bounding needs keyframe selection +
explicit loop-closure detection + a pose-graph optimizer — a whole subsystem;
keyframes are new persistent state (format change + growth management); the
occupancy grid remains a write-only second artifact (weakness #3 survives).

### (c) Hybrid: correlative coarse + point-to-grid fine refinement

Correlative search for robustness, then one Gauss–Newton polish step on the
same field for sub-cell accuracy.

Pros: best accuracy per CPU; both stages share the likelihood field.
Cons: two matchers to test and tune; the polish step is only worth it if the
benchmark shows the 5cm-quantized correlative result actually limits accuracy.

### Recommendation

**(a) correlative scan-to-map, with (c)'s refinement step as a measured,
optional follow-up.** For THIS robot: sparse gappy scans favor
correspondence-free scoring; the broken encoder demands a matcher that
survives a garbage translation prior (exhaustive window search does; gradient
descent and ICP don't); a home fits in one grid so map-anchoring alone bounds
revisit error without a pose graph; and the grid being the single persisted
artifact makes relocalization and persistence trivial. Option (b) fixes the
cost of the current matcher but not its failure modes, and imports the most
machinery.

## 4. Chosen architecture

New `GridSlam` implementing the `Slam` trait, side by side with `BasicSlam`
(selection at startup; `BasicSlam` remains default until the benchmark gate,
§6 Task 10).

### 4.1 Data structures

- **`LikelihoodField`** — `Vec<u8>` raster(s) derived from the grid, value =
  match quality of a lidar hit in that cell (255 on confident-occupied cells,
  Gaussian falloff σ ≈ 1.5 cells, 0 far from structure). Only cells with
  log-odds ≥ `CELL_OCCUPIED` seed the field — unknown and weakly-hit cells
  don't attract the matcher.
  - **Fine level**: 5cm (grid resolution), 600×600 = 360KB.
  - **Coarse level**: 20cm (4× decimation), each coarse cell = **max** of its
    fine cells, so the coarse score upper-bounds the fine score and the
    coarse-to-fine search can't discard the true optimum.
  - Rebuild: full two-pass rebuild of a 600×600 field is O(cells) and
    measured-cheap (~few ms release); rebuild lazily at most once per sweep,
    and only over the dirty bounding box of cells whose occupied-state changed
    since the last rebuild. No incremental cleverness beyond that until
    profiling demands it.
- **`MatchResult`** — best pose, normalized score ∈ [0,1] (mean per-point
  field value), number of valid points, and a 3×3 covariance estimated from
  the weighted spread of the top-scoring candidate poses (Olson's method).

### 4.2 Per-sweep pipeline (replaces `icp_match`)

1. **Predict**: propagate the previous corrected pose by the odometry delta
   (as today), yielding the search center. Deskew points with the existing
   pose-history interpolation.
2. **Window sizing**: translation window = f(odometry trust), rotation window
   = small (gyro is good). Nominal: ±0.30m / ±8°. Degraded odometry (§4.5):
   ±0.45m / ±12° around a **zero-translation** prediction. The window always
   covers worst-case inter-sweep motion, so a bad prior costs accuracy of the
   *center*, never loss of the true pose.
3. **Coarse search** (20cm / 1.5° steps): for each candidate θ, rotate the
   ~300 valid points once into grid-index offsets; each candidate translation
   is then pure array lookups. ~1k poses ≈ 370k u8 lookups.
4. **Fine search** (5cm / 0.5° steps) in a ±1-coarse-cell neighborhood of the
   coarse winner: ~700 poses ≈ 260k lookups.
   Total < 1M lookups per sweep — versus ~4.5M float distance computations
   today, and trivially within the 5–6Hz budget even in debug builds.
5. **Fuse**: precision-weighted combination of the match result and the
   odometry prediction using the match covariance (§4.4 explains why this
   matters in corridors). This replaces the ad-hoc 0.95 "odometry spring".
6. **Gate & update map**: apply the sweep to the grid via the existing
   `update_from_deskewed_scan` **only if** score ≥ `MAP_UPDATE_MIN_SCORE` or
   the local map is still immature (bootstrap: fewer than N occupied cells
   within lidar range). This is the anti-contamination valve for weakness #3's
   feedback risk.
7. **Health tracking**: EWMA of the match score feeds divergence detection
   (§4.6) and is exposed for GUI/Rerun/logs.

Bootstrap (empty map): sweep 1 is applied at the odometry pose (as today);
matching activates as soon as the field has structure. No `update_count`
special-casing survives.

### 4.3 What bounds the error

- **Within mapped space**: every match is against the global grid, so error
  does not accumulate along the trajectory; it is bounded by grid resolution +
  field smoothing (≈1–2 cells, i.e. 5–10cm) *relative to the map as drawn*.
  Revisiting a room re-registers against the walls painted on first visit —
  this is the mechanism that fixes weakness #1, and it is exactly what the
  revisit metric (§5) verifies.
- **Frontier of unmapped space**: error can grow while driving into unknown
  territory (nothing to match against ahead) and is corrected when structure
  enters view. The map-update gate prevents the transient error from painting
  confident wrong walls.
- **Explicit loop closure / pose graph: deferred.** For a single-home
  environment the map-anchored matcher keeps revisit error bounded without
  it. The known limit: if the robot maps a long chain of rooms (≫10m) with a
  weak-featured connector and the *first-pass* map is already bent, scan-to-map
  cannot un-bend it — that requires trajectory optimization. The benchmark
  perimeter-loop test measures whether we are anywhere near this limit; if a
  real house shows it, a pose-graph layer goes on top of this design rather
  than replacing it (keyframes = periodic sweep snapshots; out of scope here).

### 4.4 Feature-poor corridors

In a corridor the score surface is a ridge: cross-corridor and heading are
well constrained, along-corridor is not. The covariance estimated from the
candidate-score distribution captures exactly this (large eigenvalue along the
corridor axis). The precision-weighted fusion in step 5 then automatically
lets odometry govern the along-corridor component while the matcher pins the
other two. With the encoder faulty *and* a corridor, along-corridor drift is
genuinely unobservable — expected, reported via the health metric, and
corrected at the next corner or doorway.

### 4.5 Robustness to bad odometry (encoder fault mode)

- **Prior**: heading stays gyro-driven (already 98% gyro). Translation prior
  falls back to zero-motion; the window (±0.45m) still covers any real
  inter-sweep motion. Net effect: the matcher runs as a 5Hz local position
  tracker that needs no usable encoder translation at all.
- **Trust switch**: an explicit odometry-trust level (nominal / degraded)
  selects window sizes and prior weight. Set by config/env at minimum
  (`HR_ODOM_TRUST=low` while the hardware is broken); an automatic detector
  (encoders ticking while no drive command is active — the exact observed
  fault signature) is a cheap follow-up, flagged in the task list as a
  stretch goal.

### 4.6 Relocalization and divergence recovery

- **Restart** (fixes weakness #4/#5): `GridSlam::restore(map, pose)` rebuilds
  the likelihood field from the loaded grid and enters **relocalizing** state:
  wide window (±1.0m / ±180° coarse-first) around the saved pose, map updates
  frozen. Two consecutive sweeps with score ≥ `RELOC_CONFIRM_SCORE` confirm
  the pose and switch to normal tracking. The `update_count = 1` hack is
  deleted; dead-reckoning-until-cloud-refills no longer exists because the
  reference (the grid) is loaded, not accumulated.
- **Divergence during operation**: score EWMA below `DIVERGED_SCORE` for N
  sweeps → freeze map updates, widen the window and attempt re-acquisition
  (same code path as relocalizing). Recovery restores normal mode; sustained
  failure keeps the robot in localization-only mode and surfaces the state to
  GUI/logs (navigator integration beyond "keep working as today" is out of
  scope — see Open Questions #4).
- **Kidnap / manual move**: shows up as divergence; the widened re-acquisition
  handles small displacements. A full global relocalization (search over all
  free space × 16 headings, ~sub-second one-off in release) is designed to be
  the same matcher with the window = whole map, but is **deferred** — restart
  + local recovery cover the current operational reality.

### 4.7 Integration, trait and persistence changes

- **`Slam` trait**: unchanged in shape. One addition:
  `fn health(&self) -> SlamHealth` (score EWMA, mode: Bootstrap / Tracking /
  Relocalizing / Diverged), default-implemented so `BasicSlam` and the test
  stubs don't grow obligations. `update`'s signature stays; consumers
  (session, navigator, tests) are untouched.
- **`WorldModel`**: `pub slam: BasicSlam` becomes an enum
  `SlamEngine { Basic(BasicSlam), Grid(GridSlam) }` implementing `Slam` by
  delegation (an enum, not `Box<dyn Slam>`, because `restore`/`save` need
  concrete types and there are exactly two variants). Engine choice via env
  var `HR_SLAM=basic|grid` (same pattern as `HR_BIND`), default `basic` until
  Task 10 flips it.
- **Persistence**: **no format change required.** `HRWORLD1` (grid + pose) is
  precisely the state `GridSlam` needs; the likelihood field is derived, not
  stored. The magic stays versionable for future needs (e.g. keyframes for a
  pose graph), but this rework deliberately adds no bytes.
- **Deskewing, scan filtering (0.2–8m), grid update rules, frontier/A*/
  navigator**: unchanged. The rework swaps the matcher, not the map.

### 4.8 Dependencies

None strictly required. The correlative matcher is integer/array work; the
covariance is a 3×3 symmetric matrix (hand-rollable weighted moments +
2D/3D inverse). `nalgebra` is justified *only if* the fusion math grows past
that — decision left open (Open Questions #1). Recommendation: start
hand-rolled, keep the fusion behind a small module boundary so swapping in
`nalgebra` later is local.

## 5. Evaluation methodology (mandatory gate)

Two layers: fast deterministic Rust tests for every commit (`make ci`), and a
Gazebo ground-truth benchmark for checkpoints and the promotion gate.

### 5.1 Ground truth & data capture

- Ground truth: `gz topic -t /world/homerobot_world/pose/info -e` streamed
  (not polled) under `GZ_PARTITION=homerobot_sim`, timestamped, parsed to
  (x, y, θ) of model `homerobot`.
- SLAM estimate: the server writes a pose trace CSV when `HR_POSE_LOG=path`
  is set — one row per sweep: wall-clock time, odometry pose, SLAM pose,
  match score, mode. Env-gated, zero cost when off. (Rerun already logs
  poses, but parsing .rrd in a benchmark script is far heavier than CSV.)
- Frame alignment: robot spawns at a known world pose; the benchmark records
  the first ground-truth sample and expresses all GT relative to it, matching
  SLAM's start-at-origin convention. Residual misalignment is absorbed by the
  standard rigid alignment (Umeyama on the two trajectories) before ATE.

### 5.2 Scripted drive patterns (`tools/slam_benchmark.py`, driving via `tools/cmd_sender --proxy`)

| Pattern | Purpose |
|---|---|
| **P1 out-and-back**: 2m forward, 180°, return, 180° | sanity + RPE on straight motion |
| **P2 perimeter loop**: waypoint circuit around the arena interior (~15–20m), ending at start | drift-on-revisit — the headline test |
| **P3 room-hop**: through the interior divider/pillar area and back | matching through clutter + brief feature-poor stretches |
| **P4 = P1 with encoder fault injected** (firmware closed-loop motions run on the robot's clean encoders, so the tour is guaranteed — controlled fault/nominal comparison on identical motion) | bad-odometry robustness |
| **P5 restart-in-place**: run P1, kill server mid-run, restart, continue | relocalization |

Encoder fault injection: server-side, `HR_FAULT_RIGHT_ENCODER=1` replays the
observed signature (right delta = |right delta| + Poisson phantom ticks at
~100/s at standstill) at the `Session::handle_encoders` boundary — faithful to
issues.md #6 and independent of sim internals. (Alternative — inject in
`gazebo_bridge.cpp` — rejected: server-side works for any robot source and is
unit-testable.)

### 5.3 Metrics

- **ATE RMSE** (m): after Umeyama alignment, RMSE of positional error over
  all sweep-rate samples.
- **RPE** (m/m, °/m): relative pose error over 1m segments — drift rate
  decoupled from where it accumulated.
- **Revisit error** (m, °): pose error at the final return-to-start of P2/P4.
- **Map fidelity**: % of confident-occupied cells within 10cm of a true wall
  (arena geometry known from the world SDF), and **free-beyond-walls count**:
  confident-free cells outside the arena polygon (the sim symptom that
  motivated this rework) — must be ~0.
- **Relocalization**: sweeps-to-confirm and post-confirm pose error in P5.

### 5.4 Pass/fail thresholds

Numbers below are proposals; **Task 1 first records `BasicSlam` baselines**,
and thresholds are frozen (adjusted only downward in leniency) before any
`GridSlam` code lands, so the gate can't be tuned to fit the result.

| Metric | Threshold | Pattern |
|---|---|---|
| ATE RMSE | ≤ 0.10m (2 cells) | P1–P3 |
| RPE translation | ≤ 0.02 m/m | P1–P3 |
| RPE rotation | ≤ 0.5 °/m | P1–P3 |
| Revisit error | ≤ 0.15m, ≤ 5° after ≥15m | P2 |
| Free-beyond-walls | ≤ 25 cells (noise allowance) | P2, P3 |
| Fault-mode ATE | ≤ 2× the P1 ATE result | P4 |
| Reloc confirm | ≤ 3 sweeps, then ATE ≤ 0.10m | P5 |
| **Promotion gate** | `GridSlam` beats `BasicSlam` on ATE and revisit error on P2, and regresses no metric by >10% on any pattern | all |

### 5.5 Per-commit tests (no Gazebo)

A synthetic-world fixture in Rust: a room polygon + ray-caster generating
sweeps along a scripted trajectory, with configurable angular gap sectors
(mimicking issues.md #7), range noise, and odometry corruption. This gives
deterministic, debug-fast unit tests for the field, the matcher, gating,
relocalization and divergence — the 54-test suite grows but stays quick, and
`make ci` remains the always-green gate.

## 6. Task breakdown

> **Superseded for execution**: the authoritative, finer-grained plan (15
> resized tasks, dependency graph, per-task files and verification) is
> `docs/specs/icp-slam-rework-plan.md`. The phase structure below is the
> summary; task numbering differs.

Every task leaves `make ci` green; `BasicSlam` remains default until Task 10.

### Phase 0 — Measurement first

**Task 1: Pose trace + benchmark harness + BasicSlam baseline**
Add `HR_POSE_LOG` CSV output; write `tools/slam_benchmark.py` (GT streaming,
trajectory alignment, ATE/RPE/revisit/map-fidelity computation, P1–P3
patterns); record and commit `BasicSlam` baseline numbers to
`docs/specs/slam-baseline.md`.
*Acceptance*: benchmark runs headless on the build machine with one command;
baseline table committed; thresholds of §5.4 frozen against it.
*Verification*: `nix-shell --run "./tools/start_sim.sh --headless"` + benchmark
run; `make ci` green. *Scope*: M. *Deps*: none.

**Task 2: Encoder-fault injection (P4) + restart pattern (P5)**
`HR_FAULT_RIGHT_ENCODER` in the session encoder path with a unit test
(signature matches issues.md #6); benchmark gains P4/P5; record BasicSlam
baselines for both (expected: bad — that's the point).
*Acceptance*: fault unit test; P4/P5 baseline numbers committed.
*Verification*: `make ci`; benchmark run. *Scope*: S. *Deps*: Task 1.

**Checkpoint A**: baselines reviewed with maintainer; thresholds signed off.

### Phase 1 — Matching core (inert until selected)

**Task 3: Synthetic scan fixture**
Room-polygon ray-caster producing `LidarPoint` sweeps with gap sectors, noise,
and a scripted trajectory; used by all later unit tests.
*Acceptance*: fixture generates a P2-like loop; property test: projecting
fixture sweeps at GT poses lands ≥95% of points within 1 cell of the walls.
*Verification*: `make ci`. *Scope*: S. *Deps*: none (parallel with 1–2).

**Task 4: `LikelihoodField`**
Fine+coarse field from `OccupancyGrid` with dirty-region rebuild; coarse = max
of fine (upper bound property tested).
*Acceptance*: unit tests for falloff, occupied-only seeding, dirty-region
correctness vs full rebuild, coarse≥fine invariant; rebuild of 600×600 < 10ms
in a debug-build test.
*Verification*: `make ci`. *Scope*: M. *Deps*: none.

**Task 5: Correlative matcher**
Coarse-to-fine search, score, covariance; rotation-once/translate-by-lookup
layout.
*Acceptance*: on synthetic scans recovers injected offsets up to ±0.3m/±8°
with ≤1 fine cell / ≤0.5° error; still converges with a 30° gap sector +
noise; corridor scene yields covariance elongated along the corridor axis
(eigenvalue ratio test); full match < 20ms debug on a 600×600 map.
*Verification*: `make ci`. *Scope*: L. *Deps*: Tasks 3, 4.

**Task 6: `GridSlam` + engine selection**
`GridSlam` (predict → match → fuse → gate → map update → health), `SlamEngine`
enum in `WorldModel`, `HR_SLAM` env selection, `Slam::health()` (default impl).
*Acceptance*: all existing tests green with default `basic`; GridSlam unit
tests on synthetic loop: revisit error < BasicSlam's on the same fixture;
map-update gate provably freezes writes below threshold.
*Verification*: `make ci`; sim smoke run with `HR_SLAM=grid`. *Scope*: L.
*Deps*: Task 5.

**Checkpoint B**: benchmark P1–P3 with `HR_SLAM=grid` vs baseline; review
before robustness work.

### Phase 2 — Robustness

**Task 7: Odometry trust modes**
`HR_ODOM_TRUST` (nominal/degraded) → window sizing + zero-translation prior;
covariance-weighted fusion replacing the 0.95 spring. Stretch: automatic fault
detection (ticks while idle).
*Acceptance*: synthetic test with corrupted odometry passes tracking; P4
benchmark meets its threshold.
*Verification*: `make ci`; benchmark P4. *Scope*: M. *Deps*: Task 6.

**Task 8: Divergence detection + recovery**
Score EWMA, Diverged state, freeze-and-reacquire, health surfaced to
GUI/Rerun/log.
*Acceptance*: synthetic kidnap (teleport mid-fixture) recovers within N
sweeps with map frozen while diverged; no false positives on the clean P2
fixture. *Verification*: `make ci`. *Scope*: M. *Deps*: Task 6.

**Task 9: Restart relocalization**
Relocalizing state on `restore()`; wide-window confirm; delete the
`update_count = 1` hack.
*Acceptance*: synthetic save→restore→confirm ≤3 sweeps incl. a displaced
(±0.5m) restart; P5 benchmark meets threshold; grep confirms the hack is gone.
*Verification*: `make ci`; benchmark P5. *Scope*: M. *Deps*: Tasks 6, 8.

**Checkpoint C**: full benchmark suite (P1–P5) both engines; review.

### Phase 3 — Promotion

**Task 10: Gate + flip default**
Run the promotion gate (§5.4). If green: default `HR_SLAM=grid`, `basic`
retained behind the env var for one stabilization period; docs + issues.md
updated; real-robot session (hardware permitting) sanity check.
*Acceptance*: gate table committed; default flipped; rollback = one env var.
*Verification*: benchmark; `make ci`; manual real-robot run. *Scope*: S.
*Deps*: Checkpoint C sign-off.

### Deferred (explicitly out of scope)

- Pose-graph loop closure / keyframe store (add only if a real-house benchmark
  shows first-pass map bending — §4.3).
- Global kidnap relocalization over the whole map (design supports it; not
  needed for current operations).
- Gauss–Newton sub-cell refinement (option (c) polish — only if the benchmark
  shows 5cm quantization is the accuracy limiter).
- Automatic encoder-fault detection beyond the stretch goal (hardware repair
  is in progress).
- Adaptive lidar-gap masking in the matcher (gaps degrade score uniformly;
  revisit if P3 shows otherwise).

## 7. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Map contamination feedback locks in a bad pose | High | Update gate (§4.2.6); free-beyond-walls metric watches exactly this |
| Correlative quantization (5cm/0.5°) limits accuracy vs ICP's continuous fit | Med | Benchmark decides; deferred GN polish is the designed escape hatch |
| Field rebuild cost creeps at 5Hz | Low | Dirty-region rebuild; explicit <10ms debug test (Task 4) |
| Thresholds mis-set → gate meaningless | Med | Baselines first, thresholds frozen at Checkpoint A before matcher code exists |
| Sim benchmark passes, real robot differs (real gaps, real noise) | Med | Synthetic fixture models measured gap pattern; Task 10 includes a real-robot run |
| Divergence false-positives freeze mapping during exploration | Med | Tuned on clean fixtures (Task 8 acceptance); localization-only mode is safe-degraded, not stopped |

## 8. Open questions for the maintainer

1. **Dependency**: hand-rolled 3×3 covariance/fusion math, or take `nalgebra`
   from the start? (Recommendation: hand-rolled, module-isolated.)
2. **Engine/trust selection**: env vars (`HR_SLAM`, `HR_ODOM_TRUST`) following
   the `HR_BIND` precedent, or promote to CLI flags/config file now?
3. **Baseline thresholds**: §5.4 numbers acceptable as starting proposals to
   freeze at Checkpoint A?
4. **Navigator coupling**: should Diverged/Relocalizing pause autonomous
   exploration (navigator reads `health()`), or is GUI/log surfacing enough
   for this iteration? (Recommendation: surface only; couple later.)
5. **Pose CSV trace**: keep `HR_POSE_LOG` permanently as an env-gated
   diagnostic, or benchmark-only?
6. **Real-robot gate**: is a hardware run a hard requirement for Task 10, or
   sim-only promotion with hardware validation after the encoder repair?
