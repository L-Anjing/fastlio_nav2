#!/usr/bin/env bash
set -euo pipefail

MAP_NAME="${1:-YOUR_MAP_NAME}"
MAP_DIR="src/nav_bringup/map"
MAP_PREFIX="${MAP_DIR}/${MAP_NAME}"

if [[ "${MAP_NAME}" == "YOUR_MAP_NAME" ]]; then
  echo "Usage: ./save_grid_map.sh <map_name>"
  echo "Example: ./save_grid_map.sh lab_1"
  echo
  echo "This saves the current /map as:"
  echo "  src/nav_bringup/map/<map_name>.yaml"
  echo "  src/nav_bringup/map/<map_name>.pgm"
  exit 1
fi

mkdir -p "${MAP_DIR}"

echo "Saving FAST-LIO-SAM-G/Nav2 occupancy map to ${MAP_PREFIX}.yaml/.pgm ..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_PREFIX}"
echo "Done."
