#!/usr/bin/env bash
set -euo pipefail

# Live 单次 run：你在 RViz 点 2D Pose + Nav2 Goal；脚本负责 BT log 与评估落盘。
# Usage:
#   ./scripts/run_live_eval.sh --out results/week5d6_map1_live_run01.csv --note "W5D6 map1 goal=D live run01"
# Optional:
#   --timeout 120

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
BTLOG="/tmp/nav2_bt_live.log"

echo "== Live Eval (single run) =="
echo "[INFO] btlog: ${BTLOG}"
echo "[INFO] out:   ${OUT}"
echo "[INFO] note:  ${NOTE}"
echo

: > "${BTLOG}"
echo "[INFO] recording /behavior_tree_log -> ${BTLOG}"
echo "      (press Ctrl+C in this terminal ONLY if you want to stop recording manually)"
echo

# 用 subshell 后台起 echo，主进程跑 eval
( ros2 topic echo /behavior_tree_log >> "${BTLOG}" ) &
ECHO_PID=$!

# 让 echo 先起来，避免开跑前丢第一段 log
sleep 0.5

python3 "${ROOT_DIR}/scripts/eval_v2_action_recovery.py" \
  --btlog "${BTLOG}" \
  --timeout "${TIMEOUT}" \
  --out "${OUT}" \
  --note "${NOTE}"

# eval 退出后，关掉后台 echo
kill "${ECHO_PID}" >/dev/null 2>&1 || true

echo
echo "[OK] eval finished. csv written: ${OUT}"
