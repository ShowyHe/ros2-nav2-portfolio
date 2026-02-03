# ROS2 Nav2 Portfolio（TurtleBot3 / Humble）

一个以 **TB3 + Nav2（Humble）** 为主线的可复现学习与排障仓库：包含固定口径的评估数据、参数对照实验、跨地图泛化、bag 回放链路，以及失败案例库。

---

## Quick Start

### 环境
- Ubuntu 22.04 + ROS2 Humble
- TurtleBot3：burger
- 仿真统一：use_sim_time:=True

### Demo（当前口径：系统 launch）
终端 A（Gazebo）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

终端 B（Nav2）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

RViz 操作：
- Fixed Frame = map
- 2D Pose Estimate（设置初始位姿）
- Nav2 Goal（发送目标点）

---

## 固定口径（全仓统一）

### 成功/超时
- success：到达判据为距离目标点 <= xy_goal_tolerance
  - 基线：xy_goal_tolerance = 0.25 m（见 configs/baseline_burger.yaml -> goal_checker）
- timeout：120 s

### 碰撞（Week1 简化口径）
- collision：Week1 不做自动测量（后续周会升级为可量化口径）

---

## 配置快照（用于可复现）

本仓库运行使用系统安装包（/opt/ros/humble）。为了复现与对照，我将关键配置快照复制到仓库：

- 基线参数：configs/baseline_burger.yaml
- Map 快照：configs/map.yaml
- RViz（可选）：configs/tb3_navigation2.rviz
- 回放 QoS：configs/qos_bag_play.yaml

---

## Results（数据与总表）

- Week1 Day6 基线评估数据：results/w1d6_baseline.csv
- Week4 汇总表：results/week4_summary_table.csv
- Week5 Map1/2/3 评估数据：
  - results/week5_map1_runs.csv
  - results/week5_map2_runs.csv
  - results/week5_map3_runs.csv
- Week7 汇总目录（占位）：results/summary/

---

## Docs / Notes（文档索引）

### Week1（跑通与口径）
- Week1 Day5 复现记录：docs/week1_day5_run_repro.md
- Week1 Day6 基线评估说明：docs/week1_day6_baseline_eval.md
- Week1 Day7 复盘：docs/week1_day7_recap.md
- Week1 Day6 固定目标点截图：picture/week1_day6_ABCDpoint.png

### Week4（最小对照实验）
- Week4 Day6 总结：docs/week4_day6_summary.md
- Week4 Day7 总结：docs/week4_day7_summary.md

### Week5（泛化 + 回放）
- Week5 Day7 总结：docs/week5_day7_summary.md

---

## Failure Library（Week6）
- 入口：docs/week6_failure_index.md

### 定位方法四步法
1. 现象：用统一口径描述（S/T/F、120s、是否可视接近、恢复类型/强度）
2. 证据：最小证据集（runs 字段 + 必要的话题/TF/QoS 检查）
3. 定位：归类到根因桶（结构性可行性 / 控制层 / costmap 表征 / timebase&QoS / 定位收敛）
4. 验证：修复动作 → 对照复跑 → 指标变化作为唯一验收

---

## 目录说明（当前结构）
- configs/：参数快照、回放 QoS、RViz 配置
- docs/：按 Week 输出的过程文档与 Case
- results/：评估数据 CSV 与汇总表
- scripts/：评估脚本与统计脚本
- maps/：自建地图（pgm/yaml）
- bags/：rosbag2 数据
- picture/：关键截图（TF/costmap/planner/controller/目标点）
- videos/：演示与失败样本视频
