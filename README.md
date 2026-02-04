# ROS2 Nav2 Portfolio（TurtleBot3 / Humble）

一个以 **TB3 + Nav2（Humble）** 为主线的可复现学习与排障仓库：沉淀固定口径的评估数据、最小对照实验、跨地图泛化、bag 回放链路，以及失败案例库。

---

## Quick Start

### Demo（Nav2 系统 launch）
- 入口文档：docs/week1_day5_run_repro.md

### bag 录制 / 回放（Week5 固化流程）
- 入口文档：docs/week5_day7_summary.md

---

## 固定口径（全仓统一）

### 成功 / 超时
- success：到达判据为 **距离目标点 ≤ xy_goal_tolerance**
  - 基线：xy_goal_tolerance = 0.25 m（见 configs/baseline_burger.yaml -> goal_checker）
- timeout：120 s（固定）

### 碰撞（当前口径）
- collision：当前不做自动量化（保留为“简化口径”，后续若升级会在对应周文档单独落地）

---

## Results（数据）

- Week1 Day6 基线评估数据：results/w1d6_baseline.csv
- Week3 评估与动作记录：results/week3_day*_*.csv
- Week4 对照实验 runs：results/week4_*_runs.csv
- Week4 汇总表：results/week4_summary_table.csv
- Week5 Map1/2/3 runs：
  - results/week5_map1_runs.csv
  - results/week5_map2_runs.csv
  - results/week5_map3_runs.csv
- Week5 汇总表：results/week5_summary_table.csv
- Week5D6（Live vs Replay）：results/week5d6_*.csv

---

## Docs（总索引）
- docs/index.md

---

## Failure Library
- 入口：docs/week6_failure_index.md

---

## 目录说明（当前结构）
- assets/：辅助资源（如 urdf）
- bags/：rosbag2 数据
- configs/：参数快照、回放 QoS、RViz 配置
- docs/：按 Week 输出的过程文档与 Case
- maps/：自建地图（pgm/yaml）
- picture/：关键截图（TF/costmap/planner/controller/目标点）
- results/：评估数据 CSV 与汇总表
- scripts/：评估脚本与统计脚本
- videos/：演示
