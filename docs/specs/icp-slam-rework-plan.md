# Implementation Plan: ICP/SLAM Quality Rework

## Overview

Execution plan for the design in `docs/specs/icp-slam-rework.md`: replace the
rolling-cloud ICP in `BasicSlam` with map-anchored correlative scan matching
(`GridSlam`), measured against ground truth before promotion. This document
supersedes the coarser task list in the design doc's §6 — same phases, tasks
resized so nothing is larger than M.

**Ground rules (from the design):**
- Every task leaves `make ci` green (54 tests + clippy `-D warnings`).
- `BasicSlam` stays the default engine until Task 15's benchmark gate.
- Baselines are recorded and thresholds frozen (Checkpoint A) before any
  matcher code exists.

## Architecture Decisions (settled in the design doc)

- Correlative scan-to-map matching on a likelihood field derived from the
  log-odds grid; no kd-tree ICP, no pose graph (deferred).
- No new dependencies; hand-rolled 3×3 covariance/fusion math, module-isolated.
- Engine selection via `SlamEngine` enum in `WorldModel` + `HR_SLAM` env var
  (follows the `HR_BIND` precedent).
- No persistence format change: `HRWORLD1` (grid + pose) is sufficient.
- Fault injection server-side (`HR_FAULT_RIGHT_ENCODER`), not in the sim bridge.

## Dependency Graph

```
T1 pose trace ──┐
T2 benchmark ───┼── T5 baselines ══ Checkpoint A (thresholds frozen)
T3 patterns ────┤
T4 fault inj ───┘
                          T6 synthetic fixture ──┐
                          T7 likelihood field ───┼── T8 coarse matcher ── T9 fine+covariance
                                                 │                              │
                                                 └── T10 GridSlam core ◄────────┘
                                                          │
                                                     T11 engine selection ══ Checkpoint B
                                                          │
                                        ┌─────────────────┼─────────────────┐
                                   T12 odom trust    T13 divergence    T14 relocalization
                                        └─────────────────┼─────────────────┘
                                                    Checkpoint C
                                                          │
                                                   T15 promotion gate
```

Vertical slices: each phase ends in a *benchmarkable capability* — (0) we can
measure, (1) the new engine tracks, (2) it survives faults/restarts, (3) it's
the default. T6 is parallel-safe with all of Phase 0.

## Task List

### Phase 0: Measurement first

#### Task 1: Pose trace logging (`HR_POSE_LOG`)
**Description:** When `HR_POSE_LOG=<path>` is set, the session appends one CSV
row per processed sweep: wall-clock seconds, odometry pose (x, y, θ), SLAM pose
(x, y, θ), match score, engine mode. Score/mode are placeholders (`0.0`,
`basic`) until `Slam::health()` exists (T11) — column layout fixed now so the
benchmark parser never changes.
**Acceptance criteria:**
- [ ] CSV with header row written only when the env var is set; rows appended
      per sweep in `process_sweep`
- [ ] Unit test: sweeps through a test session produce parseable rows
- [ ] Zero cost / no file touched when unset
**Verification:** `make ci`
**Dependencies:** None
**Files:** `server/src/session.rs`, `server/src/main.rs` (env plumbing)
**Scope:** S

#### Task 2: Benchmark harness (`tools/slam_benchmark.py`)
**Description:** Python harness: streams ground truth (`gz topic -e`,
`GZ_PARTITION=homerobot_sim`), reads the `HR_POSE_LOG` CSV, time-aligns the two
series, applies Umeyama rigid alignment, computes ATE RMSE, RPE (1m segments,
translation + rotation), revisit error, and map fidelity (wall-distance % and
free-beyond-walls count from `house_map.bin` + arena polygon from the world
SDF). Emits a machine-readable JSON result + human table.
**Acceptance criteria:**
- [ ] All §5.3 metrics computed from a recorded run, headless
- [ ] Metric unit-check: synthetic CSV with known injected drift yields the
      expected ATE/RPE within tolerance (pure-python test, no Gazebo)
- [ ] JSON output suitable for later gate comparison (T15)
**Verification:** metric self-test locally; one manual-drive run on the build
machine end-to-end
**Dependencies:** Task 1 (CSV format)
**Files:** `tools/slam_benchmark.py`
**Scope:** M

#### Task 3: Scripted drive patterns P1–P3
**Description:** Waypoint scripts driven through `tools/cmd_sender --proxy`:
P1 out-and-back (2m, 180°, return), P2 perimeter loop (~15–20m circuit ending
at start), P3 room-hop through the divider/pillar area. Integrated as
`slam_benchmark.py --pattern p1|p2|p3` so a full run is one command.
**Acceptance criteria:**
- [ ] Each pattern runs unattended in the headless sim and completes
- [ ] P2 verifiably returns within 0.5m of start per ground truth (drive
      quality, not SLAM quality)
**Verification:** three benchmark runs on the build machine
**Dependencies:** Task 2
**Files:** `tools/slam_benchmark.py` (or `tools/patterns/*.py` if it grows)
**Scope:** S

#### Task 4: Encoder fault injection + restart pattern (P4, P5)
**Description:** `HR_FAULT_RIGHT_ENCODER=1` corrupts right-encoder deltas at
the `Session::handle_encoders` boundary, replaying the issues.md #6 signature
(delta → |delta|, plus phantom ticks ~100/s at standstill, deterministic RNG).
Benchmark gains P4 (= P2 + fault) and P5 (P1 with a mid-run server
kill/restart, asserting map reload + continued tracking).
**Acceptance criteria:**
- [ ] Unit test: injected stream shows always-positive right deltas and
      nonzero standstill ticks; disabled ⇒ passthrough
- [ ] P4 and P5 run unattended
**Verification:** `make ci`; P4/P5 runs on the build machine
**Dependencies:** Tasks 1, 3
**Files:** `server/src/session.rs`, `tools/slam_benchmark.py`
**Scope:** M

#### Task 5: BasicSlam baselines + threshold freeze
**Description:** Run P1–P5 (3 repetitions each) with `BasicSlam`, commit the
result tables to `docs/specs/slam-baseline.md`, and freeze the §5.4 pass/fail
thresholds against them (adjust only toward leniency, with rationale).
**Acceptance criteria:**
- [ ] Baseline tables committed with sim/world/commit provenance
- [ ] Threshold table marked FROZEN in the design doc
**Verification:** benchmark runs; human sign-off
**Dependencies:** Tasks 1–4
**Files:** `docs/specs/slam-baseline.md`, `docs/specs/icp-slam-rework.md`
**Scope:** S

### Checkpoint A (after Tasks 1–5)
- [ ] `make ci` green; benchmark reproducible (≤10% metric variance across reps)
- [ ] Baselines + frozen thresholds reviewed by maintainer
- [ ] No matcher code exists yet (gate integrity)

### Phase 1: Matching core (inert until selected)

#### Task 6: Synthetic scan fixture
**Description:** Test-only module: room polygon(s) + 2D ray-caster generating
`LidarPoint` sweeps along a scripted trajectory, with configurable angular gap
sectors (issues.md #7 pattern), range noise, and odometry corruption. The
shared substrate for T7–T14 unit tests; keeps debug-build tests Gazebo-free
and fast.
**Acceptance criteria:**
- [ ] Generates a P2-like loop trajectory with sweeps at 5Hz spacing
- [ ] Property test: points projected at ground-truth poses land ≥95% within
      1 cell of the fixture walls
**Verification:** `make ci`
**Dependencies:** None (parallel with Phase 0)
**Files:** `server/src/test_fixtures.rs` (`#[cfg(test)]`), `server/src/main.rs`
(mod decl)
**Scope:** S

#### Task 7: `LikelihoodField`
**Description:** Fine (5cm) + coarse (20cm, max-decimated) `u8` rasters built
from the grid: cells with log-odds ≥ `CELL_OCCUPIED` seed value 255, Gaussian
falloff σ≈1.5 cells. Dirty-region rebuild driven by a change-tracking hook on
occupied-state transitions; rebuilt lazily at most once per sweep.
**Acceptance criteria:**
- [ ] Unit tests: falloff shape, occupied-only seeding, coarse ≥ fine
      upper-bound invariant, dirty-region result ≡ full rebuild
- [ ] Full 600×600 rebuild < 10ms in a debug-build test
**Verification:** `make ci`
**Dependencies:** None
**Files:** `server/src/likelihood.rs` (new), `server/src/mapping.rs`
(change-tracking hook), `server/src/main.rs` (mod decl)
**Scope:** M

#### Task 8: Correlative matcher — coarse search
**Description:** Coarse-level exhaustive search: for each candidate θ (1.5°
steps), rotate valid scan points once into grid-index offsets; score each
candidate translation (20cm steps) by summed field lookups. Returns
best-scoring pose + normalized score. Window is a parameter (T12 sizes it).
**Acceptance criteria:**
- [ ] Recovers injected offsets up to ±0.45m / ±12° to within one coarse cell
      / one θ step on fixture scans
- [ ] Still converges with a 30° gap sector + range noise
- [ ] Coarse pass < 10ms debug on a 600×600 map
**Verification:** `make ci`
**Dependencies:** Tasks 6, 7
**Files:** `server/src/matcher.rs` (new), `server/src/main.rs` (mod decl)
**Scope:** M

#### Task 9: Correlative matcher — fine refinement + covariance
**Description:** Fine-level search (5cm / 0.5°) in a ±1-coarse-cell
neighborhood of the coarse winner; `MatchResult` gains the 3×3 covariance from
the weighted spread of top candidates (hand-rolled moments), used by T10's
fusion and T12's corridor handling.
**Acceptance criteria:**
- [ ] End-to-end match error ≤ 1 fine cell / ≤ 0.5° on fixture offsets
- [ ] Corridor fixture yields covariance elongated along the corridor axis
      (eigenvalue ratio > 5)
- [ ] Full coarse+fine match < 20ms debug
**Verification:** `make ci`
**Dependencies:** Task 8
**Files:** `server/src/matcher.rs`
**Scope:** M

#### Task 10: `GridSlam` core
**Description:** New engine implementing the per-sweep pipeline: odometry
predict (reusing the existing deskew/pose-history code paths) → match → 
precision-weighted fusion with the prior → map-update gate
(`MAP_UPDATE_MIN_SCORE`, bootstrap exemption) → grid update → score EWMA.
Implements the `Slam` trait; handles `on_odometry_reset` like `BasicSlam`.
Not yet reachable from production wiring.
**Acceptance criteria:**
- [ ] Fixture P2-loop revisit error strictly better than `BasicSlam` on the
      identical fixture (side-by-side unit test)
- [ ] Gate test: sweeps scoring below threshold leave the grid byte-identical
- [ ] Bootstrap test: empty map ⇒ first sweep applied at odometry pose,
      matching active by sweep 3
**Verification:** `make ci`
**Dependencies:** Tasks 7, 9 (fixture via 6)
**Files:** `server/src/slam_grid.rs` (new), `server/src/main.rs` (mod decl)
**Scope:** M

#### Task 11: Engine selection + `Slam::health()`
**Description:** `SlamEngine { Basic, Grid }` enum in `WorldModel` delegating
the `Slam` trait; `HR_SLAM=basic|grid` (default `basic`); `Slam::health()`
with a default impl so `BasicSlam`/test stubs are untouched; T1's CSV columns
now populated from `health()`; `WorldModel::load`/`save`/`restore` route to
the active engine.
**Acceptance criteria:**
- [ ] Full suite green with default `basic` (zero behavior change)
- [ ] Session-level test: `HR_SLAM=grid` world processes sweeps and persists/
      restores through `house_map.bin`
- [ ] Pose CSV rows carry real score/mode under grid engine
**Verification:** `make ci`; sim smoke run with `HR_SLAM=grid` on the build
machine
**Dependencies:** Task 10
**Files:** `server/src/world.rs`, `server/src/slam.rs`, `server/src/session.rs`,
`server/src/main.rs`
**Scope:** M

### Checkpoint B (after Tasks 6–11)
- [ ] `make ci` green, default engine unchanged
- [ ] Benchmark P1–P3 with `HR_SLAM=grid`: results vs baseline reviewed
      (not yet required to pass all thresholds — direction check)
- [ ] Maintainer review before robustness work

### Phase 2: Robustness

#### Task 12: Odometry trust modes
**Description:** `HR_ODOM_TRUST=nominal|degraded` selects window sizes
(±0.30m/±8° vs ±0.45m/±12°) and prior (odometry vs zero-translation +
gyro-heading); fusion weights driven by the T9 covariance — the 0.95 "spring"
constant is gone from the grid path. Stretch (separate commit, may be
dropped): auto-degrade when encoders tick with no active drive command.
**Acceptance criteria:**
- [ ] Fixture test with issues.md #6-style corrupted odometry: degraded mode
      keeps revisit error within 2× the clean-odometry result
- [ ] Corridor fixture: along-corridor component follows the prior,
      cross-corridor follows the matcher (covariance-weighting test)
- [ ] P4 benchmark meets its frozen threshold
**Verification:** `make ci`; benchmark P4
**Dependencies:** Task 11
**Files:** `server/src/slam_grid.rs`, `server/src/matcher.rs`,
`server/src/main.rs`
**Scope:** M

#### Task 13: Divergence detection + recovery
**Description:** Score-EWMA state machine: Tracking → Diverged (below
`DIVERGED_SCORE` for N sweeps: freeze map writes, widen window, re-acquire) →
Tracking on recovery; sustained failure stays localization-only. Health state
surfaced to GUI status + Rerun + log.
**Acceptance criteria:**
- [ ] Fixture kidnap (mid-trajectory teleport ≤ 0.5m) recovers within 5
      sweeps; grid frozen (byte-identical) while Diverged
- [ ] No false Diverged transitions across the clean P2 fixture
**Verification:** `make ci`
**Dependencies:** Task 11
**Files:** `server/src/slam_grid.rs`, `server/src/session.rs`,
`server/src/gui/mod.rs` (status line)
**Scope:** M

#### Task 14: Restart relocalization
**Description:** `GridSlam::restore(map, pose)` enters Relocalizing: field
rebuilt from the loaded grid, wide window (±1.0m, ±180° coarse-first) around
the saved pose, map frozen until two consecutive sweeps ≥
`RELOC_CONFIRM_SCORE`. Deletes the `update_count = 1` hack (grid path;
`BasicSlam` keeps its behavior until removal).
**Acceptance criteria:**
- [ ] Fixture: save → restore → confirmed ≤ 3 sweeps, including a restart
      displaced by ±0.5m / ±30°
- [ ] Failure path: restore against a wrong map stays Relocalizing, never
      writes the grid
- [ ] P5 benchmark meets its frozen threshold
**Verification:** `make ci`; benchmark P5
**Dependencies:** Tasks 11, 13 (shares the re-acquisition path)
**Files:** `server/src/slam_grid.rs`, `server/src/world.rs`
**Scope:** M

### Checkpoint C (after Tasks 12–14)
- [ ] Full benchmark suite P1–P5, both engines, results committed
- [ ] `make ci` green
- [ ] Maintainer review before promotion

### Phase 3: Promotion

#### Task 15: Promotion gate + default flip
**Description:** Run the frozen gate: `GridSlam` beats `BasicSlam` on ATE and
revisit error on P2 and regresses no metric >10% on any pattern. If green:
default `HR_SLAM=grid`, `basic` retained behind the env var; update README /
design doc / issues.md; real-robot sanity session **after** the encoder repair
(per design decision — sim-only promotion is acceptable).
**Acceptance criteria:**
- [ ] Gate comparison table committed (JSON from T2 + human table)
- [ ] Default flipped; rollback documented as one env var
- [ ] If the gate FAILS: results + analysis committed, default stays `basic`,
      follow-up tasks filed — failing the gate is a valid outcome
**Verification:** benchmark suite; `make ci`
**Dependencies:** Checkpoint C sign-off
**Files:** `server/src/world.rs` (default), `README.md`, docs
**Scope:** S

## Parallelization

- **Safe in parallel:** T6 with all of Phase 0; T2/T3 (Python) with T1 (Rust);
  T7 with T6; T12/T13 after T11.
- **Sequential:** T8→T9→T10 (one module chain); T14 after T13 (shares the
  re-acquisition path); everything gates on Checkpoint A for threshold
  integrity.
- **Contract-first:** T1 fixes the CSV column layout before T2 consumes it and
  T11 populates it.

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Map contamination feedback locks in a bad pose | High | Update gate (T10) + free-beyond-walls metric (T2) watches exactly this |
| Benchmark variance makes the gate noisy | Med | 3 reps at T5; ≤10% variance required at Checkpoint A before thresholds freeze |
| Correlative quantization limits accuracy vs continuous ICP fit | Med | Benchmark decides; Gauss–Newton polish is the designed, deferred escape hatch |
| Field rebuild cost creeps at 5Hz | Low | Dirty-region rebuild + explicit <10ms debug test (T7) |
| Sim-only promotion hides real-lidar quirks | Med | Fixture models the measured gap pattern (T6); hardware session scheduled post-repair (T15) |
| Divergence false-positives freeze mapping mid-exploration | Med | T13 false-positive test on clean fixture; Diverged is safe-degraded, not stopped |
| Scope creep into pose-graph territory | Med | Deferred list in design doc §6 is explicit; P2 revisit metric tells us if we ever actually need it |

## Open Questions

Carried from the design doc §8 with the defaults adopted on 2026-07-02:
hand-rolled math (no `nalgebra`), env-var configuration, health surfaced to
GUI/logs only (no navigator coupling), sim-only promotion with hardware
validation after the encoder repair. Remaining for the maintainer:

1. Are the §5.4 threshold numbers acceptable to freeze at Checkpoint A?
2. Keep `HR_POSE_LOG` permanently as an env-gated diagnostic, or strip it
   after promotion?
