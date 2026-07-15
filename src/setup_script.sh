#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# This script can be run either from the workspace root or from the src folder.
# It searches both likely locations for the simulation package.
CANDIDATE_DIRS=(
  "$SCRIPT_DIR/simulation"
  "$SCRIPT_DIR/src/simulation"
)

SIM_DIR=""
for dir in "${CANDIDATE_DIRS[@]}"; do
  if [[ -d "$dir" ]]; then
    SIM_DIR="$dir"
    break
  fi
done

if [[ -z "$SIM_DIR" ]]; then
  echo "simulation package not found next to this script or under ./src"
  echo "Nothing to mark executable."
  exit 0
fi

mark_executable_if_present() {
  local path="$1"
  if [[ -f "$path" ]]; then
    chmod +x "$path"
    echo "Marked executable: $path"
  else
    echo "Not found, skipping: $path"
  fi
}

mark_executable_if_present "$SIM_DIR/scripts/run_unity.sh"
mark_executable_if_present "$SIM_DIR/unity_sim/Build_Ubuntu/AD_Sim.x86_64"

# Mark shell scripts in simulation/scripts as executable.
if [[ -d "$SIM_DIR/scripts" ]]; then
  find "$SIM_DIR/scripts" -maxdepth 1 -type f -name "*.sh" -print0 | while IFS= read -r -d '' script; do
    chmod +x "$script"
    echo "Marked executable: $script"
  done
fi

echo "Setup script finished."
echo "Next steps:"
echo "  cd <your_ros2_workspace>"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build"
echo "  source install/setup.bash"
