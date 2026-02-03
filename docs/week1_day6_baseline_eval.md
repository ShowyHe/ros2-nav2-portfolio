# Week1 Day6 — 基线评估（Baseline Evaluation）

## 评估口径（固定）
- Goal：从 bt_navigator 日志中解析 `to (x, y)`（map frame）
- Final：从 `/amcl_pose` 读取最终位姿（map frame）
- FinalDist：goal 与 final 的欧氏距离
- Success：FinalDist ≤ 0.25 m
- Timeout：120 s

## 固定目标点（A/B/C/D）
截图：
- picture/week1_day6_ABCDpoint.png

| Goal | (x, y) |
|---|---|
| A | (0.00, 1.80) |
| B | (1.09, 0.53) |
| C | (0.56, -0.54) |
| D | (1.30, -1.62) |

## 结果
- 数据文件：results/w1d6_baseline.csv
- 成功率：4 / 12 = 33.3%
- 分目标成功率：
  - A：0/3
  - B：1/3
  - C：0/3
  - D：3/3
