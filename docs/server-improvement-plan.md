# Implementation Plan: Server Hardening After Event-Model Refactor

## Overview

Follow-up to the `refactor/server-event-model` branch. Four goals, in order: (1) lock in
the refactor by making the session testable and covered, (2) fix the known navigation
correctness bugs, (3) make the SLAM map survive robot reconnects and server restarts,
(4) stand up CI so all of this stays green. Sourced from the full-codebase review of
2026-07-02.

## Architecture Decisions

- **Session testability via generics, not mocks**: `ProtocolManager` is already generic
  over `Read + Write`; extend the same approach to `RobotLink`/`Session` so tests drive
  the session with in-memory buffers. Drop the unused `mockall` dependency rather than
  adopt it.
- **World model as a shared, session-independent component**: `Slam` + map move out of
  `Session` into a `WorldModel` owned by `main`, passed to sessions as `Arc<Mutex<_>>`.
  Session remains the only writer during a connection, so lock contention is trivial.
- **Costmap precomputation over per-node inflation checks**: A* moves to an inflated
  costmap built once per replan instead of O(r²) `is_obstacle_free` per neighbor.
- **CI runs on the existing build machine** (rsync + nix-shell flow proven on
  2026-07-02), since the repo's shell.nix does not evaluate on macOS. GitHub Actions can
  come later if the repo gets a hosted remote.

## Task List

### Phase 1: Lock in the refactor (testability + hygiene)

#### Task 1: Make Session testable over in-memory streams
**Description:** Make `RobotLink` generic over `W: Write` (mirroring `ProtocolManager`)
and `Session` generic over its link, so `handle_event` can run in unit tests against a
`Vec<u8>` sink. No behavior change.
**Acceptance criteria:**
- [ ] `Session::handle_event` runs in a test with no TCP socket
- [ ] Production code path (`run_session`) unchanged in behavior
**Verification:** remote `cargo test` green; manually run server + simulation once.
**Dependencies:** None
**Files:** `server/src/session.rs`
**Scope:** S

#### Task 2: Session unit tests
**Description:** Cover the event loop end-to-end: encoder payload updates pose and emits
`GuiUpdate`s; `SetMode` triggers planning; RPC request/response round-trip correlates by
`call_id`; a manual `Drive` cancels autonomous mode; `Reset` clears state and emits
`ResetView`; duplicate drive commands are deduped on the wire.
**Acceptance criteria:**
- [ ] ≥6 tests covering the flows above, asserting on emitted bytes and `GuiUpdate`s
- [ ] A test proving an RPC reply with the wrong `call_id` does NOT complete a pending motion
**Verification:** remote `cargo test` green.
**Dependencies:** Task 1
**Files:** `server/src/session.rs` (test module)
**Scope:** M

#### Task 3: ProtocolManager framing tests
**Description:** Unit-test the wire framing against byte slices: message split across
reads, garbage length prefix, message larger than `BUFFER_SIZE`, protobuf decode failure,
clean EOF.
**Acceptance criteria:**
- [ ] ≥5 tests, including the split-read reassembly case
**Verification:** remote `cargo test` green.
**Dependencies:** None (parallel with Task 1)
**Files:** `server/src/reader.rs`
**Scope:** S

#### Task 4: Dependency and dead-code cleanup
**Description:** Remove unused deps (`mockall`, `signal-hook`, `libc`); delete dead code
kept alive by `#[allow(dead_code)]` (`Odometry::update` → rewrite its test via
`update_encoders`, `OccupancyGrid::update_from_scan`, `get_cell`); decide fate of
`stats.rs` counters (surface `total_rx/tx` in GUI status or delete them).
**Acceptance criteria:**
- [ ] `cargo build` emits zero warnings
- [ ] No `#[allow(dead_code)]` remains in server/src
**Verification:** remote `cargo test` green, warning-free build.
**Dependencies:** None
**Files:** `server/Cargo.toml`, `odometry.rs`, `mapping.rs`, `stats.rs`, `gui/mod.rs`
**Scope:** S

### Checkpoint A (after Tasks 1-4)
- [ ] All tests pass on build machine, zero warnings
- [ ] Simulation smoke test: connect, drive manually, start exploration
- [ ] Human review before Phase 2

### Phase 2: Navigation correctness

#### Task 5: Shared occupancy thresholds
**Description:** Replace the scattered magic numbers (`< -10` frontier-free, `<= -20`
planner-free, `< -5`/`> 10` PGM export) with named constants in `mapping.rs` used by
`pathfinding.rs`, `find_frontiers` and `save_pgm`. Align frontier-reachability with the
planner's definition of free so frontiers are plannable by construction.
**Acceptance criteria:**
- [ ] One source of truth for FREE/OCCUPIED thresholds
- [ ] Test: every detected frontier on a synthetic map is reachable by `plan_path`
**Verification:** remote `cargo test`; exploration run in simulation reaches frontiers
without blacklist churn.
**Dependencies:** None
**Files:** `mapping.rs`, `pathfinding.rs`
**Scope:** S

#### Task 6: Mapping unit tests
**Description:** Tests for `world_to_grid` round-trip and boundary cells, `raytrace_free`
endpoints (must not clear the hit cell), log-odds clamping, and `find_frontiers` on a
hand-built grid (room with one opening → exactly one frontier at the opening).
**Acceptance criteria:**
- [ ] ≥5 tests as above
**Verification:** remote `cargo test` green.
**Dependencies:** Task 5 (uses the named constants)
**Files:** `mapping.rs`
**Scope:** S

#### Task 7: A* correctness and costmap performance
**Description:** Fix inadmissible heuristic (Manhattan + diagonals): move to octile
heuristic with 10/14 step costs; add closed-set/stale-node skip; precompute an inflated
costmap once per `plan_path` call instead of `is_obstacle_free` per neighbor expansion.
**Acceptance criteria:**
- [ ] Test: straight-line path in open space has no zigzag (path length optimal ±1 cell)
- [ ] Test: path around an obstacle respects the inflation radius
- [ ] Test: unreachable goal returns `None` within a bounded time
- [ ] `plan_path` on a 600×600 map with obstacles completes < 50ms in a debug-build test
**Verification:** remote `cargo test`; watch replan latency in simulation logs.
**Dependencies:** Task 5
**Files:** `pathfinding.rs`
**Scope:** M

#### Task 8: Stuck detection measures progress toward goal
**Description:** In `Navigator`, progress = distance-to-goal decreasing (with small
hysteresis), not raw pose delta — a robot spinning in place no longer resets the stuck
timer. Keep rotation as progress only during the initial alignment phase (large
`angle_diff`).
**Acceptance criteria:**
- [ ] Test: oscillating rotation with constant distance-to-goal triggers stuck within
      `PROGRESS_TIMEOUT`
- [ ] Test: slow but monotonic approach never triggers stuck
**Verification:** remote `cargo test`; simulation: robot boxed in by obstacle blacklists
the frontier and moves on.
**Dependencies:** None
**Files:** `navigator.rs`
**Scope:** S

### Checkpoint B (after Tasks 5-8)
- [ ] Full exploration run in simulation completes without manual intervention
      — BLOCKED by issues.md #3 (sim right wheel inverted, robot cannot translate);
      unit tests cover the nav pipeline in the meantime
- [ ] Human review before Phase 3

> Status 2026-07-02: Phase 1 done (commits 2ba9c87, 4fd7d6f, acfc7b2, 773d135 +
> sim smoke test on pcluca). Phase 2 done (412d234 thresholds+mapping tests,
> 2ebd5b5 A* rewrite, 1d5d9be stuck detection). 46 tests green, zero warnings.
> Hardware findings recorded in issues.md.

### Phase 3: Persistent world model

#### Task 9: Extract WorldModel from Session
**Description:** Move `BasicSlam` (and current pose estimate) into a `WorldModel` created
in `main` and shared with sessions via `Arc<Mutex<_>>`. A reconnecting robot resumes with
the existing map and pose instead of starting from scratch. `Reset` command clears the
shared model explicitly.
**Acceptance criteria:**
- [ ] Session test: disconnect + new session sees the previous map and pose
- [ ] `Reset` still zeroes everything
**Verification:** remote `cargo test`; simulation: kill robot process mid-exploration,
restart it, verify map continuity in GUI.
**Dependencies:** Tasks 1-2 (session tests exist to catch regressions)
**Files:** `session.rs`, `slam.rs`, `main.rs`, new `world.rs`
**Scope:** M

#### Task 10: Map persistence across server restarts
**Description:** Periodically autosave the raw log-odds grid (not the lossy PGM) plus the
current pose to disk; load on startup if present. PGM export stays as a human-readable
snapshot. Decide format: simple bincode/postcard of `(width, height, resolution, Vec<i16>)`.
**Acceptance criteria:**
- [ ] Round-trip test: save → load → identical grid
- [ ] Server restart resumes with the saved map (manual check)
- [ ] Autosave interval configurable, default ~30s, skipped when map unchanged
**Verification:** remote `cargo test`; restart server during simulation, map survives.
**Dependencies:** Task 9
**Files:** `world.rs`, `mapping.rs`, `main.rs`
**Scope:** M

#### Task 11: Single active robot session
**Description:** Enforce one active robot connection: a new connection replaces the old
one (log + close previous). Prevents two sessions writing the shared world model and
duplicated command execution.
**Acceptance criteria:**
- [ ] Test/manual: second connection closes the first cleanly, GUI status stays coherent
**Verification:** manual: connect simulator twice.
**Dependencies:** Task 9
**Files:** `main.rs`, `session.rs`
**Scope:** S

### Checkpoint C (after Tasks 9-11)
- [ ] Reconnect + restart resilience demonstrated in simulation
- [ ] Human review before Phase 4

### Phase 4: CI and polish

#### Task 12: CI on the build machine
**Description:** Script (`tools/ci.sh`) encapsulating the proven flow: rsync
`server/ proto/ Makefile shell.nix` to `luca@192.168.199.120:~/HomeRobot-ci/`, run
`cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test` in nix-shell, report
status. Wire into a git pre-push hook or cron; document in README.
**Acceptance criteria:**
- [ ] One command runs the full check suite remotely and returns nonzero on failure
- [ ] Clippy is clean (fix findings or annotate deliberately)
**Verification:** run it; break a test intentionally and confirm it fails.
**Dependencies:** Phases 1-3 merged (clippy fixes touch everything)
**Files:** `tools/ci.sh`, `README.md`
**Scope:** S

#### Task 13: Throttle map GUI updates
**Description:** Stop cloning the full 720KB grid per sweep: send `GuiUpdate::Map` at
most every N sweeps or when changed, and skip all GUI sends when the receiver is closed
(headless).
**Acceptance criteria:**
- [ ] Map update allocation ≤ 1/s in steady state
- [ ] Headless mode performs zero map clones
**Verification:** log allocation counters in a debug run.
**Dependencies:** None
**Files:** `session.rs`, `gui/mod.rs`
**Scope:** S

## Deferred (explicitly out of scope for this plan)

- ICP grid-bucketed nearest-neighbor / scan-to-map matching and loop closure (real work,
  needs its own spec; current matcher is functional)
- High-level `ExecuteMotion` commands to remove the `ROBOT_SIZES` global and the 5×
  duplicated tick math (worthwhile, but touches GUI + proxy + firmware contract)
- GUI module split (cosmetic)
- Firmware unit tests (Zephyr ztest; separate toolchain)
- GitHub Actions (no confirmed hosted remote; build-machine CI covers the need)

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Threshold alignment (Task 5) changes exploration behavior on real maps | Med | Simulation run at Checkpoint B before merging; thresholds in one place makes tuning easy |
| WorldModel extraction reintroduces shared-mutable-state bugs the refactor removed | Med | Session stays sole writer; lock scope reviewed at Checkpoint C; session tests from Task 2 guard regressions |
| A* rewrite changes paths in ways the follower dislikes | Low | Path-quality tests + sim run; follower uses lookahead so smoother paths should only help |
| Build machine unavailable → no CI/tests | Low | Local `cargo check` still possible after GTK libs installed; document brew fallback |

## Open Questions

1. Map persistence format: plain custom binary vs `postcard`/`bincode` dependency — any
   preference? (Task 10)
2. Should a *new* robot connection replace the old session (proposed) or be rejected?
   (Task 11)
3. Is there a hosted git remote (GitHub?) worth wiring CI to instead of the build
   machine?
