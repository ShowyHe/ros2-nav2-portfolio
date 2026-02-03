# Week1 Day5 — Nav2 TB3 Demo（跑通 + 可复现）

## 目标
在 TurtleBot3 Gazebo 仿真中跑通 Nav2，并固定 success/timeout 的口径，作为后续评估与对照实验的统一基准。

## 环境
- Ubuntu 22.04（WSL）
- ROS2 Humble
- TurtleBot3：burger
- use_sim_time:=True（Gazebo 时钟）

## 证据
- 视频：videos/week1_day5_demo.mp4

## 我使用的系统入口（/opt/ros/humble）
本次 Demo 使用系统安装包启动：

- Launch：
  - /opt/ros/humble/share/turtlebot3_navigation2/launch/navigation2.launch.py
- 默认参数（burger）：
  - /opt/ros/humble/share/turtlebot3_navigation2/param/humble/burger.yaml
- Map：
  - /opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
- RViz：
  - /opt/ros/humble/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz

为便于复现与后续对照，我将关键快照复制到仓库：
- configs/baseline_burger.yaml
- configs/map.yaml
- （可选）configs/tb3_navigation2.rviz

## Week1 固定口径
- success：到达判据为距离目标点 <= xy_goal_tolerance
  - 基线：xy_goal_tolerance = 0.25 m（见 configs/baseline_burger.yaml -> goal_checker）
- timeout：120 s
- collision：Week1 简化（暂不自动测量，后续周升级为可量化口径）

## 复现启动方式（与 Day5 Demo 一致）
终端 A（Gazebo）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

终端 B（Nav2）：

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

RViz 操作：
1) Fixed Frame = map
2) 2D Pose Estimate（设置初始位姿）
3) Nav2 Goal（发送目标点）
