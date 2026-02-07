# ROS2 Nav2 Portfolio（TurtleBot3 / Humble）

[English](README_EN.md) | [中文](README.md)

一个以 **TB3 + Nav2（Humble）** 为主线的可复现学习与排障仓库：沉淀固定口径的评估数据、最小对照实验、跨地图泛化、bag 回放链路，以及失败案例库。

---

## Quick Start

### Demo（Nav2 系统 launch）
- 入口文档：docs/week1_day5_run_repro.md

### bag 录制 / 回放（Week5 固化流程）
- 入口文档：docs/week5_day7_summary.md

---

## 8 周完成内容总览（Week1–Week8）

本仓库按周递进把“能跑起来”推进到“可复现、可量化、可定位、可沉淀”。

### Week1：跑通 + 基线数据落地
- 完成：Nav2 Demo 跑通，冻结 success/timeout 基本口径。
- 产出：复现文档 + 基线评估数据（固定目标点、可回看）。

### Week2：链路解释与证据化
- 完成：把 TF / costmap / planner-controller / BT recovery 的关键点讲清楚，并以参数与观测证据落盘。
- 产出：对应模块的证据文档（参数、话题/可视化检查口径）。

### Week3：评估证据链升级（Eval v0 → v2）
- 完成：从手工记录升级为可复现的评估链路，记录 success/time 与恢复行为。
- 产出：Eval v0/v1/v2 的演进与结果数据；最小 A/B 对照实验数据。

### Week4：最小对照实验（同图同起终点，用数据对比）
- 完成：同一地图、固定起终点下，对 6 组配置做对照（Baseline / A / B(theta=0.6) / B'(theta=0.4) / A+B(theta=0.6) / A+B'(theta=0.4)）。
- 产出：runs 数据 + 汇总表（summary_table）+ 可解释结论文档。

### Week5：三地图泛化压力 + bag 回放闭环
- 完成：用 Map1/2/3 测泛化压力；打通 bag 录制/回放；recovery 证据链闭环（不依赖主观观察）。
- 产出：三地图 runs 与汇总表；Live vs Replay 对照；回放/录制流程文档。

### Week6：失败案例库沉淀（5 类根因桶）
- 完成：将典型失败按统一结构沉淀为可检索 Case（现象—证据—根因—修复—验证）。
- 产出：5 个 Case 文档 + 失败索引页（failure index）。

### Week7：仓库收敛为作品集结构（入口与索引）
- 完成：README 保持“一屏入口”；所有细节由 docs/index 与周文档承接，不引入新的启动方式与脚本。
- 产出：docs 总索引完善；Week7 打包说明文档。

### Week8：Ready Packet（证据绑定总入口）
- 完成：将 Week1–Week7 的能力与证据收敛为“一页总入口”，方便快速复核与回顾。
- 产出：docs/week8_ready_packet.md。

---

## 八周最终交付是什么？

**一个可复现的 Nav2 学习与排障作品集仓库**，包含：
- 权威数据入口（基线与汇总表）+ 逐次 runs 原始记录；
- 按周沉淀的过程文档（链路解释、评估、对照、泛化、回放）；
- 失败案例库（统一结构、可检索）；
- 清晰入口结构：README（一屏入口） + docs/index（总索引） + Week8 Packet（总入口）。

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
