# ROS2 Nav2 Portfolio（TurtleBot3 / Humble）

以 **TB3 + Nav2（Humble）** 为主线的可复现学习仓库：包含统一口径的评估数据、参数对照实验、跨地图泛化、bag 回放链路，以及失败案例库。

---

## Quick Start（一屏入口）

### 0）环境
- Ubuntu 22.04 + ROS2 Humble
- TurtleBot3：burger
- 仿真统一：`use_sim_time:=True`

### 1）Nav2 Demo（系统 launch 口径）
- 复现入口脚本：`./scripts/run_demo.sh`
- 需要自定义地图时：在终端 B 的 launch 里加 `map:=<yaml>`（示例见下）

终端 A（Gazebo）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

终端 B（Nav2）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

指定仓库内地图（示例为map3）：

    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
      use_sim_time:=True \
      map:=/home/hexiaoyi/ros2_nav2_portfolio/maps/week5_map3_narrow_house.yaml

RViz 操作口径：
- Fixed Frame = `map`
- 2D Pose Estimate（设置初始位姿）
- Nav2 Goal（发送目标点）

---

## 统一口径（全仓固定）

### 成功 / 超时
- success：到达判据为 **距离目标点 ≤ `xy_goal_tolerance`**
  - 基线：`xy_goal_tolerance = 0.25 m`（见 `configs/baseline_burger.yaml` → goal_checker）
- timeout：`120 s`

### 碰撞（当前口径）
- Week1：不做自动测量（后续以“可量化口径”升级并写入结果表）

---

## 可复现快照（对照/留痕用）

本仓库运行使用系统安装包（`/opt/ros/humble`）。为便于复现与对照，关键配置已复制到仓库：
- 基线参数快照：`configs/baseline_burger.yaml`
- Map 快照（系统 map）：`configs/map.yaml`
- RViz（可选）：`configs/tb3_navigation2.rviz`
- 回放 QoS：`configs/qos_bag_play.yaml`

---

## Evaluation（一键入口）

### Live 单次评估（一次 run 自动落 CSV）
> 在 RViz 点 2D Pose + Nav2 Goal；脚本负责 BT log + 评估落盘。

    ./scripts/run_live_eval.sh --timeout 120 \
      --out results/week5d6_map1_live_run01.csv \
      --note "W5D6 map1 goal=D live run01"

### Bag 回放评估（伪真机输入）
> 先用脚本打印回放命令（按输出开终端），再跑单次评估落盘。

    ./scripts/print_bag_replay_cmds.sh --bag bags/week5d6_map1_batch01

（回放评估脚本入口在输出中：`./scripts/run_bag_replay_eval.sh ...`）

---

## Results（数据入口）

- Week1 Day6 基线评估：`results/w1d6_baseline.csv`
- Week4 2×2 对照汇总表：`results/week4_summary_table.csv`
- Week5 Map1/2/3 评估数据：
  - `results/week5_map1_runs.csv`
  - `results/week5_map2_runs.csv`
  - `results/week5_map3_runs.csv`
- Week7 汇总目录（占位）：`results/summary/`

---

## Docs（总索引）
- 文档总索引：`docs/index.md`

---

## Failure Library（Week6）
- 入口：`docs/week6_failure_index.md`

定位方法四步法（全仓统一）：
1. 现象：统一口径描述（S/T/F、120s、是否可视接近、恢复类型/强度）
2. 证据：最小证据集（runs 字段 + 必要的话题/TF/QoS 检查）
3. 定位：归类到根因桶（结构性可行性 / 控制层 / costmap 表征 / timebase&QoS / 定位收敛）
4. 验证：修复动作 → 对照复跑 → 指标变化作为唯一验收

---

## 目录结构（当前）
- `configs/`：参数快照、回放 QoS、RViz 配置
- `docs/`：按 Week 输出的过程文档与 Case（索引见 `docs/index.md`）
- `results/`：评估 CSV 与汇总表（汇总目录：`results/summary/`）
- `scripts/`：评估脚本与统计脚本
- `maps/`：自建地图（pgm/yaml）
- `bags/`：rosbag2 数据
- `picture/`：关键截图（TF/costmap/planner/controller/目标点）
- `videos/`：演示
