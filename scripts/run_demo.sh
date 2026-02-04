#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./scripts/run_demo.sh
#   ./scripts/run_demo.sh --map maps/week5_map3_narrow_house.yaml
#   ./scripts/run_demo.sh --map configs/map.yaml

ROS_SETUP="/opt/ros/humble/setup.bash"
TB3_MODEL="burger"

MAP_ARG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map)
      MAP_ARG="$2"
      shift 2
      ;;
    *)
      echo "[ERR] unknown arg: $1"
      exit 1
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERR] missing: ${ROS_SETUP}"
  exit 1
fi

if [[ -n "${MAP_ARG}" && ! -f "${MAP_ARG}" ]]; then
  echo "[ERR] map file not found: ${MAP_ARG}"
  exit 1
fi

echo "== Nav2 Demo (TB3) =="
echo
echo "Terminal A (Gazebo):"
echo "  source ${ROS_SETUP}"
echo "  export TURTLEBOT3_MODEL=${TB3_MODEL}"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo
echo "Terminal B (Nav2):"
echo "  source ${ROS_SETUP}"
echo "  export TURTLEBOT3_MODEL=${TB3_MODEL}"
if [[ -n "${MAP_ARG}" ]]; then
  echo "  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(realpath "${MAP_ARG}")"
else
  echo "  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True"
fi
echo
echo "RViz 操作口径：Fixed Frame=map -> 2D Pose Estimate -> Nav2 Goal"
