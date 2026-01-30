# Week5 Day4 — 三地图泛化对比汇总（Map1 vs Map2 vs Map3）

## 1. 今日目标
- 把 Map1 / Map2 / Map3 的 runs.csv 做成**可对比的总表**（成功率 / TIMEOUT 比例 / 耗时 / recovery_hits）
- 输出“泛化结论”：哪张最稳、哪张最吃力、差异体现在哪里（用数据说话）
- 特别说明：Map3 的 TIMEOUT（T）已通过 notes 追加 `VISUAL=reached / VISUAL=not_reached`，避免把 T 直接等价成“失败”

---

## 2. 评估口径
- timeout：120s
- 固定脚本：scripts/eval_v2_action_recovery.py
- `result` 字段统一解释：
  - `S`：timeout 内捕捉到终态为 SUCCEEDED
  - `F`：timeout 内捕捉到终态为 FAILED/ABORTED/CANCELED 等“明确失败终态”
  - `T`：120s 内未捕捉到终态（统一记为 TIMEOUT ）
- `recovery_hits`：
  - 从 BT log 的窗口 `[bt_start_line, bt_end_line)` 内，按 configs/recovery_keywords.txt 做关键词命中次数统计
  - hits 口径 **≠ RViz 面板里 recoveries 次数**（统计对象不同）

---

## 3. 输入（地图与点位）
- Map1：turtlebot3_world（基线）
- Map2：week5_narrow_house（窄通道）
- Map3：start=(1, 3)，goal=(6.15, -1.15)（障碍/局部规划更易卡住的场景）

---

## 4. 三地图汇总总表（来源：results/week5_summary_table.csv）
| map | runs | S/T/F | success_rate | timeout_rate | fail_rate | median_time(s) | p90_time(s) | median_hits | p90_hits | max_hits |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Map1 | 20 | 20/0/0 | 1.00 | 0.00 | 0.00 | 27.2435 | 35.5642 | 0 | 2.0 | 2 |
| Map2 | 20 | 15/4/1 | 0.75 | 0.20 | 0.05 | 60.2210 | 120.0000 | 2 | 13.3 | 31 |
| Map3 | 20 | 1/19/0 | 0.05 | 0.95 | 0.00 | 120.0000 | 120.0000 | 18 | 18.2 | 20 |

补充：hits 的 P90 为分位数计算结果，可能出现小数（插值导致），不影响对比判断。

---

## 5. Map3 的 TIMEOUT（T）拆解：VISUAL 标注结果
- Map3 TIMEOUT 总数：19 / 20 = 95%
- 在 TIMEOUT 样本里：
  - `VISUAL=reached`：16 / 19 = 84.21%
  - `VISUAL=not_reached`：3 / 19 = 15.79%

解释：
- `VISUAL=reached` 占多数，说明 Map3 的 T **不能简单等价为“机器人没到”**；更像是“终态未被脚本在 120s 内捕捉到”或“到达后仍在恢复/状态链路未归档终态”。
- `VISUAL=not_reached` 的那部分，才更接近“确实没到终点”的 TIMEOUT。

---

## 6. 结论
### 结论 1：Map1 最稳（基线场景）
- 证据：
  - success_rate = 1.00（20/20）
  - median_time = 27.24s，p90_time = 35.56s
  - median_hits = 0（大多数 run 不依赖恢复）
- 解释：
  - 开阔环境 + 路径可行性高，局部规划更少陷入恢复链，因此整体更“干净”。

### 结论 2：Map2 能跑但更“工程味”——恢复依赖显著上升
- 证据：
  - success_rate = 0.75（15/20），timeout_rate = 0.20（4/20），fail_rate = 0.05（1/20）
  - median_time = 60.22s（耗时明显上升），p90_time = 120s（尾部风险大）
  - median_hits = 2，p90_hits ≈ 13.3，max_hits = 31（恢复强烈介入）
- 解释：
  - 窄通道更容易触发 ClearCostmap/Spin/BackUp 等恢复动作；因此“能到的样本也更慢”，尾部样本更容易拖到 120s。

### 结论 3：Map3 主要表现为 TIMEOUT 桶占比极高 + hits 长期高位
- 证据：
  - success_rate = 0.05（1/20），timeout_rate = 0.95（19/20）
  - median_time = 120s（几乎都撞到上限）
  - median_hits = 18，max_hits = 20（恢复链路几乎持续运行）
  - TIMEOUT 中 84% 为 `VISUAL=reached`（到达但未归档终态/状态未被捕捉是主要矛盾）
- 解释（当前阶段以“可解释”优先，不做过度推断）：
  - Map3 更容易陷入“恢复循环”或“终态捕捉链路不完整”的状态：表现为 hits 极高且持续、同时 120s 内没有明确终态。
  
---

## 7. 口径差异说明：为什么 RViz recoveries=9，但脚本 hits=18
- RViz 的 recoveries 更像“恢复阶段/触发次数”的 UI 汇总
- 脚本的 hits 是“日志关键词命中次数”：
  - 同一恢复周期内同一关键词可能出现多次（例如 ClearLocalCostmap 在一个周期内多次打印）
- 结论：
  - 两者不矛盾；本周统一以 runs.csv 的 recovery_hits 做可复现对比口径。

---

## 8. 证据链索引
- Map1 原始数据：results/week5_map1_runs.csv
- Map2 原始数据：results/week5_map2_runs.csv
- Map3 原始数据：results/week5_map3_runs.csv
- 三图汇总表：results/week5_summary_table.csv
- Day2 记录：docs/week5_day2_map1_20runs.md
- Day3 记录：docs/week5_day3_generalization_map2_map3.md

---

## 9. Week5 Day5方向
- 固化 bag 回放“伪真机”流程：record/play --clock + use_sim_time 全链一致 + 一键复现
- 目标不是跑满次数，而是把流程写成“别人复制就能跑”的工程文档
