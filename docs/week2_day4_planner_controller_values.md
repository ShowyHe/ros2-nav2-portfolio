# Week2 Day4 — Planner / Controller 实验 + 排查剧本（Nav2 TB3）

目标：用 3 个可复现实验，把 Planner vs Controller 的责任边界讲清楚，并形成 5 步排查剧本。

---

## 0. 今日交付物（本地落库）
- docs/week2_day4_planner_controller_values.md
- docs/week2_day4_playbook_planner_controller.md
- picture/planner_controller/
  - week2_day4_planner_baseline.png
  - week2_day4_planner_big_inflation.png
  - week2_day4_controller_max_vel_theta_small.png
  - week2_day4_controller_goal_tolerance_tight.png

---

## 1. 一句话分工
- Planner：用 global_costmap 规划全局路径（/plan 或 /plan_smoothed）。
- Controller：沿全局路径生成速度命令 /cmd_vel（受 local_costmap 与约束影响）。

---

## 2. 系统事实
### 2.1 Topic
- /plan
- /plan_smoothed
- /local_plan
- /cmd_vel
- /cmd_vel_nav

### 2.2 插件
- planner_plugins: ['GridBased']
- controller_plugins: ['FollowPath']
- GridBased.plugin: nav2_navfn_planner/NavfnPlanner

### 2.3 Goal / Progress
- general_goal_checker.xy_goal_tolerance = 0.25
- general_goal_checker.yaw_goal_tolerance = 0.25
- progress_checker.required_movement_radius = 0.5
- progress_checker.movement_time_allowance = 10.0

---

## 3. 三个对照实验

### 实验 A（Planner）：global inflation 改变 → 路径形态改变
#### A1) Baseline（global_costmap）
- inflation_radius = 0.55
- cost_scaling_factor = 3.0
截图：picture/planner_controller/week2_day4_planner_baseline.png

#### A2) Big inflation（global_costmap）
- inflation_radius = 2.5
- cost_scaling_factor = 1.0
截图：picture/planner_controller/week2_day4_planner_big_inflation.png

#### A3) 结论（一句话）
Planner 会读取 global_costmap 的代价场；inflation 变大/衰减更慢时，路径更保守、更远离障碍，但通常仍能规划出路径。

---

### 实验 B（Controller）：限制角速度 → 转不动/卡住/拖很久
#### B1) 原值
- FollowPath.max_vel_theta = 1.0
- FollowPath.min_speed_theta = 0.0

#### B2) 实验设置
- FollowPath.max_vel_theta = 0.1
- FollowPath.min_speed_theta = 0.0
截图：picture/planner_controller/week2_day4_controller_max_vel_theta_small.png

#### B3) 结论
有路径但走不起来/原地很久，优先怀疑 Controller 约束；角速度上限太小会导致转向效率极低 → 任务拖长/卡住。

---

### 实验 C（Controller）：目标容忍度极小 → 到点附近抖/不结束
#### C1) 原值（证据）
- general_goal_checker.xy_goal_tolerance = 0.25
- general_goal_checker.yaw_goal_tolerance = 0.25

#### C2) 实验设置
- general_goal_checker.xy_goal_tolerance = 0.02
- general_goal_checker.yaw_goal_tolerance = 0.02
截图：picture/planner_controller/week2_day4_controller_goal_tolerance_tight.png

#### C3) 结论
有路径也在走但一直 active/到不了 SUCCEEDED，优先看 goal_checker；容忍度太小会导致目标附近无限微调/振荡/不结束。

---

## 4. 参数回滚清单
### Controller 回滚
- FollowPath.max_vel_theta = 1.0
- FollowPath.min_speed_theta = 0.0
- general_goal_checker.xy_goal_tolerance = 0.25
- general_goal_checker.yaw_goal_tolerance = 0.25

### Global costmap 回滚
- /global_costmap/global_costmap inflation_layer.inflation_radius = 0.55
- /global_costmap/global_costmap inflation_layer.cost_scaling_factor = 3.0

---

## 5. 今日结论
- 看 /plan：Planner 负责“能不能规划出路”，受 global_costmap 代价场影响；inflation 变大通常让路更保守，不必然 No valid path。
- 看 /cmd_vel：Controller 负责“能不能走起来/走得顺”；角速度太小会转不动卡住；goal tolerance 太小会到点附近振荡不结束。
---
