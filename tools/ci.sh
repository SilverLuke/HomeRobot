#!/usr/bin/env bash
# CI for the Rust server: syncs the sources to the build machine and runs
# clippy (deny warnings) + the test suite inside the project nix-shell.
#
# The server cannot be built on macOS (shell.nix does not evaluate on darwin,
# GTK4 native libs unavailable), so this drives the Linux build machine.
#
#   ./tools/ci.sh            # clippy + tests
#   ./tools/ci.sh --fmt      # additionally check rustfmt (currently opt-in:
#                            # the codebase is not rustfmt-formatted yet)
#
# Config via env:
#   HR_CI_HOST  ssh target             (default luca@192.168.199.120)
#   HR_CI_DIR   remote work directory  (default ~/HomeRobot-ci)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
HOST="${HR_CI_HOST:-luca@192.168.199.120}"
REMOTE_DIR="${HR_CI_DIR:-HomeRobot-ci}"

CHECK_FMT=false
for arg in "$@"; do
    [ "$arg" = "--fmt" ] && CHECK_FMT=true
done

echo "== Syncing sources to ${HOST}:${REMOTE_DIR} =="
rsync -az --delete \
    --exclude 'target' --exclude 'logs' \
    "${PROJECT_ROOT}/server" "${PROJECT_ROOT}/proto" "${PROJECT_ROOT}/shell.nix" \
    "${HOST}:${REMOTE_DIR}/"

FMT_CMD=""
if [ "$CHECK_FMT" = true ]; then
    FMT_CMD="echo '== rustfmt ==' && cargo fmt --check &&"
fi

echo "== Running clippy + tests on ${HOST} =="
# shellcheck disable=SC2029
ssh "$HOST" "cd ${REMOTE_DIR}/server && nix-shell ../shell.nix --run \"
    set -e
    ${FMT_CMD} true
    echo '== clippy (deny warnings) =='
    cargo clippy --all-targets -- -D warnings
    echo '== tests =='
    cargo test
\"" 2>&1 | grep -vE "^(evaluation warning|                    )"

echo "== CI PASSED =="
