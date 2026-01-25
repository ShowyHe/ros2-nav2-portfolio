#!/usr/bin/env python3
import argparse
import csv
import os
from typing import List, Tuple, Set

def read_lines(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        return f.readlines()

def load_keywords(path: str) -> List[str]:
    kws = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            kws.append(line)
    return kws

def detect_recovery(lines: List[str], keywords: List[str]) -> Tuple[str, str, int]:
    """
    Return:
      recovery_YN: 'Y' if any keyword hit, else 'N'
      recovery_types: semicolon-joined unique hits (stable order)
      hit_count: total hits (not unique)
    """
    hit_types: List[str] = []
    total = 0
    for ln in lines:
        for kw in keywords:
            if kw in ln:
                total += 1
                hit_types.append(kw)

    uniq: List[str] = []
    seen: Set[str] = set()
    for x in hit_types:
        if x not in seen:
            uniq.append(x)
            seen.add(x)

    recovery = "Y" if total > 0 else "N"
    types = ";".join(uniq)
    return recovery, types, total

def slice_by_line_range(all_lines: List[str], start_1based: int, end_1based: int) -> List[str]:
    n = len(all_lines)
    s = max(1, start_1based)
    e = min(end_1based, n)
    if e < s:
        return []
    return all_lines[s-1:e]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--btlog", default="/tmp/w2d6_bt.log")
    ap.add_argument("--keywords", default="configs/recovery_keywords.txt")
    ap.add_argument("--out", default="results/week3_day1_eval.csv")
    args = ap.parse_args()

    if not os.path.exists(args.btlog):
        raise SystemExit(f"BT log not found: {args.btlog}")
    if not os.path.exists(args.keywords):
        raise SystemExit(f"keywords file not found: {args.keywords}")

    keywords = load_keywords(args.keywords)
    all_lines = read_lines(args.btlog)

    print("W3D1 eval_v0 — 半自动评估记录器（BT log 切片 + recovery 命中）")
    print(f"BT log: {args.btlog} (lines={len(all_lines)})")
    print("每次 run：输入 run_id / start_line / end_line / result / time_sec / notes")
    print("脚本：在 start~end 之间自动判定 recovery 和 recovery_types")
    print("结束：run_id=0")

    # 确保 results/ 存在（你现在已经有，但这里防御一下）
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    rows = []
    while True:
        run_id = input("\nrun_id (1..10, 0=finish): ").strip()
        if not run_id:
            continue
        if run_id == "0":
            break

        start_line = int(input("start_line (1-based): ").strip())
        end_line = int(input("end_line (1-based): ").strip())
        result = input("result (S/T/F): ").strip().upper()
        time_sec = input("time_sec (number): ").strip()
        notes = input("notes (short): ").strip()

        # refresh lines because file grows
        all_lines = read_lines(args.btlog)
        chunk = slice_by_line_range(all_lines, start_line, end_line)

        recovery, types, hit_count = detect_recovery(chunk, keywords)

        rows.append({
            "run": run_id,
            "result": result,
            "time_sec": time_sec,
            "recovery": recovery,
            "recovery_types": types,
            "recovery_hits": str(hit_count),
            "start_line": str(start_line),
            "end_line": str(end_line),
            "notes": notes,
        })

        print(f"→ recovery={recovery}, types={types if types else '-'}, hits={hit_count}, slice_lines={len(chunk)}")

    fieldnames = ["run","result","time_sec","recovery","recovery_types","recovery_hits","start_line","end_line","notes"]
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    print(f"\nSaved: {args.out} (rows={len(rows)})")

if __name__ == "__main__":
    main()
