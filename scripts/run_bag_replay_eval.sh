#!/usr/bin/env bash
set -euo pipefail

# Bag replay 单次 run：要求你已启动 bag play (Terminal A) + Nav2 (Terminal B)。
# 你在 RViz 点 2D Pose + Nav2 Goal；脚本负责 BT log 与评估落盘。
# Usage:
#   ./scripts/run_bag_replay_eval.sh --out results/xxx.csv --note "..."

TIMEOUT="120"
OUT=""
NOTE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) TIMEOUT="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --note) NOTE="$2"; shift 2;;
    *) echo "[ERR] unknown arg: $1"; exit 1;;
  esac
done

if [[ -z "${OUT}" || -z "${NOTE}" ]]; then
  echo "[ERR] require: --out <csv> --note <text>"
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BTLOG="/tmp/nav2_bt_replay.log"

: > "${BTLOG}"
( ros2 topic echo /behavior_tree_log >> "${BTLOG}" ) &
ECHO_PID=$!
sleep 0.5

python3 "${ROOT_DIR}/scripts/eval_v2_action_recovery.py" \
  --btlog "${BTLOG}" \
  --timeout "${TIMEOUT}" \
  --out "${OUT}" \
  --note "${NOTE}"

kill "${ECHO_PID}" >/dev/null 2>&1 || true
echo "[OK] replay eval finished: ${OUT}"
