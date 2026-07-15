#!/usr/bin/env bash
set -euo pipefail
PKG_PREFIX="$(ros2 pkg prefix simulation)"
UNITY_BIN="${PKG_PREFIX}/share/simulation/unity_sim/Build_Ubuntu/AD_Sim.x86_64"
exec "${UNITY_BIN}" "$@"
