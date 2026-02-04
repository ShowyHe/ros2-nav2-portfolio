#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./scripts/print_bag_replay_cmds.sh --bag bags/week5d6_map1_batch01

ROS_SETUP="/opt/ros/humble/setup.bash"
TB3_MODEL="burger"
DOMAIN="88"
BAG=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2;;
    *) echo "[ERR] unknown arg: $1"; exit 1;;
  esac
done

if [[ -z "${BAG}" ]]; then
  echo "[ERR] require: --bag <bag_dir>"
  exit 1
fi

if [[ ! -d "${BAG}" ]]; then
  echo "[ERR] bag dir not found: ${BAG}"
  exit 1
fi

echo "== Bag Replay Commands =="
echo
echo "Terminal A (bag play):"
echo "  source ${ROS_SETUP}"
echo "  export ROS_DOMAIN_ID=${DOMAIN}"
echo "  cd $(pwd)"
echo "  ros2 bag play ${BAG} --loop \\"
echo "    --topics /clock /tf /tf_static /scan /map /odom /behavior_tree_log"
echo
echo "Terminal B (Nav2):"
echo "  source ${ROS_SETUP}"
echo "  export ROS_DOMAIN_ID=${DOMAIN}"
echo "  export TURTLEBOT3_MODEL=${TB3_MODEL}"
echo "  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True"
echo
echo "Terminal C (eval single run):"
echo "  ./scripts/run_bag_replay_eval.sh --out results/week5d6_bag_replay_run01.csv --note \"W5D6 replay map1 goal=D run01\""
