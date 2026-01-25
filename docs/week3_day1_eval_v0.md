# Week3 Day1 — Eval v0 (from BT log)

## 做了什么
把 W2D6 的“10 次手工表格”升级为“半自动评估流水线 v0”。

## 输入
- /tmp/w2d6_bt.log：用 ros2 topic echo /behavior_tree_log 持续写入

## 人工填写字段
- result: S/T/F
- time_sec: 秒表计时
- notes: 一句话标签（clean / oscillation / stuck...）

## 脚本自动字段
- recovery: Y/N（在 start_line~end_line 切片内命中关键词）
- recovery_types: 命中的关键词去重列表
- recovery_hits: 命中次数（非去重）

## 输出
- results/week3_day1_eval.csv

## 为什么需要 start_line/end_line
为了只统计“本次 run 的增量日志”，避免把历史 recovery 误算进本次 run。

## Summary
- runs=10, success=10/10 (100.0%), avg_time=15.81s, median_time=15.35s, recovery_runs=0
- recovery_runs=0 表示 10 次导航均未触发清图/倒车/旋转等恢复动作（评估集为“纯净成功样本”），后续 W3D2/W3D3 再加入“故障注入”验证恢复链路。
