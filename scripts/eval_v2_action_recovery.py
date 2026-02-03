#!/usr/bin/env python3
import argparse
import csv
import os
import time
from typing import List, Tuple, Set, Optional

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray

ACTIVE = {1, 2, 3}      # ACCEPTED/EXECUTING/CANCELING
TERMINAL = {4, 5, 6}    # SUCCEEDED/CANCELED/ABORTED


def ns_now(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds)


def goal_uuid_hex(goal_id) -> str:
    # Humble: goal_id.uuid is uint8[16] (often numpy.ndarray)
    try:
        return bytes(goal_id.uuid).hex()
    except Exception:
        return "".join(f"{int(b):02x}" for b in goal_id.uuid)


def load_keywords(path: str) -> List[str]:
    kws = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            kws.append(ln)
    return kws


def read_lines(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return f.readlines()


def count_recovery_hits(lines: List[str], keywords: List[str]) -> Tuple[str, str, int]:
    hits = []
    total = 0
    for ln in lines:
        for kw in keywords:
            if kw in ln:
                total += 1
                hits.append(kw)

    uniq = []
    seen: Set[str] = set()
    for x in hits:
        if x not in seen:
            uniq.append(x)
            seen.add(x)

    recovery = "Y" if total > 0 else "N"
    return recovery, ";".join(uniq), total


def next_run_id(csv_path: str) -> int:
    """
    更稳：读取最后一条数据行的 run，run+1。
    避免用“行数”导致手工插入/删行后 run 重复。
    """
    if not os.path.exists(csv_path):
        return 1

    last_run = 0
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("run,"):
                continue
            # 数据行：run 在第一列
            parts = ln.split(",", 1)
            try:
                last_run = int(parts[0])
            except Exception:
                pass

    return last_run + 1 if last_run > 0 else 1


class EvalV2(Node):
    def __init__(self, args):
        super().__init__("eval_v2_action_plus_recovery")
        self.args = args

        self.btlog = args.btlog
        self.keywords = load_keywords(args.keywords)

        self.goal_id: Optional[str] = None
        self.start_ns: Optional[int] = None
        self.end_ns: Optional[int] = None
        self.result: Optional[str] = None

        self.bt_start_line = self._bt_line_count()

        self.row_written = False

        self.sub = self.create_subscription(
            GoalStatusArray,
            args.status_topic,
            self.on_status,
            10
        )

        self.get_logger().info("W3D3 eval_v2: 启动成功")
        self.get_logger().info(f"Action status topic: {args.status_topic}")
        self.get_logger().info(f"BT log: {self.btlog}")
        self.get_logger().info(f"bt_start_line={self.bt_start_line}")
        self.get_logger().info("现在去 RViz 点一次 Nav2 Goal（先启动脚本，再点）")

    def _bt_line_count(self) -> int:
        if not os.path.exists(self.btlog):
            return 0
        with open(self.btlog, "r", encoding="utf-8", errors="ignore") as f:
            return sum(1 for _ in f)

    def on_status(self, msg: GoalStatusArray):
        # 第一次：锁定一个 active goal 并开始计时
        if self.goal_id is None:
            for st in msg.status_list:
                if st.status in ACTIVE:
                    self.goal_id = goal_uuid_hex(st.goal_info.goal_id)
                    self.start_ns = ns_now(self)
                    self.get_logger().info(f"捕捉到 goal_id={self.goal_id}，开始计时…")
                    return

        # 后续：只跟踪这个 goal，直到终态
        if self.goal_id is not None and self.result is None:
            for st in msg.status_list:
                gid = goal_uuid_hex(st.goal_info.goal_id)
                if gid != self.goal_id:
                    continue
                if st.status in TERMINAL:
                    self.end_ns = ns_now(self)
                    if st.status == 4:
                        self.result = "S"
                    elif st.status == 5:
                        self.result = "T"
                    else:
                        self.result = "F"
                    self.write_row()   # 终态正常写入
                    return

    def write_row(
        self,
        result_override: Optional[str] = None,
        time_sec_override: Optional[float] = None,
        note_suffix: str = ""
    ):
        if self.row_written:
            return
        self.row_written = True

        # 结束行：以当前 btlog 总行数为准
        all_lines = read_lines(self.btlog) if os.path.exists(self.btlog) else []
        bt_end_line = len(all_lines)
        chunk = all_lines[self.bt_start_line:bt_end_line]

        recovery, types, hits = count_recovery_hits(chunk, self.keywords)

        # result / time
        final_result = result_override or self.result or ""
        if time_sec_override is not None:
            time_sec = f"{time_sec_override:.3f}"
        else:
            time_sec = ""
            if self.start_ns and self.end_ns and self.end_ns > self.start_ns:
                time_sec = f"{(self.end_ns - self.start_ns)/1e9:.3f}"

        # notes
        base_note = self.args.note or ""
        final_note = (base_note + ((" | " + note_suffix) if note_suffix else "")).strip()

        out = self.args.out
        os.makedirs(os.path.dirname(out), exist_ok=True)
        file_exists = os.path.exists(out)

        run_id = next_run_id(out)

        fieldnames = [
            "run", "goal_id", "result", "time_sec", "notes",
            "recovery", "recovery_types", "recovery_hits",
            "bt_start_line", "bt_end_line"
        ]

        with open(out, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                w.writeheader()
            w.writerow({
                "run": run_id,
                "goal_id": self.goal_id or "",
                "result": final_result,
                "time_sec": time_sec,
                "notes": final_note,
                "recovery": recovery,
                "recovery_types": types,
                "recovery_hits": str(hits),
                "bt_start_line": str(self.bt_start_line),
                "bt_end_line": str(bt_end_line),
            })

        self.get_logger().info(
            f"[logged] run={run_id} result={final_result} time_sec={time_sec} "
            f"recovery={recovery} types={types if types else '-'} hits={hits}"
        )
        self.get_logger().info(f"Output: {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--btlog", default="/tmp/w3d3_bt.log")
    ap.add_argument("--keywords", default="configs/recovery_keywords.txt")
    ap.add_argument("--out", default="results/week3_day3_eval_v2.csv")
    ap.add_argument("--note", default="")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--status_topic", default="/navigate_to_pose/_action/status")
    args = ap.parse_args()

    rclpy.init()
    node = EvalV2(args)

    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

            # 终态写入后就退出
            if node.row_written and node.result is not None:
                break

            # 超时：也写入一条
            if time.time() - t0 > args.timeout and not node.row_written:
                if node.goal_id is None:
                    node.get_logger().error("超时：未捕捉到 goal_id（请确保先启动脚本，再点 Nav2 Goal）")
                    node.write_row(
                        result_override="T",
                        time_sec_override=args.timeout,
                        note_suffix="TIMEOUT:no_goal_id"
                    )
                else:
                    node.get_logger().error("超时：已捕捉到 goal_id，但未等到终态（可能导航耗时>timeout）")
                    node.write_row(
                        result_override="T",
                        time_sec_override=args.timeout,
                        note_suffix="TIMEOUT:no_terminal_status"
                    )
                break

    except KeyboardInterrupt:
        # 你手动 Ctrl+C：也尽量落一条，免得白跑
        if not node.row_written:
            if node.goal_id is None:
                node.write_row(result_override="T", time_sec_override=args.timeout, note_suffix="INTERRUPT:no_goal_id")
            else:
                node.write_row(result_override="T", time_sec_override=args.timeout, note_suffix="INTERRUPT:goal_no_terminal")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
