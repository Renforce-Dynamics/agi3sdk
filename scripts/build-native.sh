#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$TASK_ROOT" -B "$TASK_ROOT/build" -DBUILD_TESTING=ON "$@"
cmake --build "$TASK_ROOT/build" --parallel
ctest --test-dir "$TASK_ROOT/build" --output-on-failure
