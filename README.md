# Nav2 TB3 Demo — Week1 Day5 (Run + Reproducible)

## Goal
Run Nav2 in TurtleBot3 Gazebo simulation and freeze success/timeout criteria for later evaluation.

## Environment
- Ubuntu 22.04 (WSL)
- ROS2 Humble
- TurtleBot3: burger
- use_sim_time:=True (Gazebo clock)

## Evidence
- Video: videos/week1_day5_demo.mp4

## What I ran (SYSTEM files)
This demo is launched using system-installed packages under /opt/ros/humble:

- Launch entry:
  /opt/ros/humble/share/turtlebot3_navigation2/launch/navigation2.launch.py
- Default params used by the system launch (burger):
  /opt/ros/humble/share/turtlebot3_navigation2/param/humble/burger.yaml
- Map used by the system launch:
  /opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml
- RViz config:
  /opt/ros/humble/share/turtlebot3_navigation2/rviz/tb3_navigation2.rviz

For reproducibility, I copied snapshots into this repo:
- configs/baseline_burger.yaml
- configs/map.yaml
- (optional) configs/tb3_navigation2.rviz

## Fixed criteria (Week1)
- success: robot is considered reached when distance to goal <= xy_goal_tolerance
  - baseline: xy_goal_tolerance = 0.30 m (see configs/baseline_burger.yaml -> goal_checker)
- timeout: 120 s
- collision: simplified (Week1 not auto-measured; will be upgraded later)

## How to run (same as my Day5 demo)

### Terminal A (Gazebo)
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
### Terminal B (Nav2)
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

### RViz steps
1) Fixed Frame = map
2) 2D Pose Estimate (set initial pose)
3) Nav2 Goal (send goal)