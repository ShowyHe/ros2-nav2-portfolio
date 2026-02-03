# Week4 Day4 — B 组对照实验（FollowPath.max_vel_theta）

## 实验目的
在固定地图与固定起终点口径下，仅调整 Controller（FollowPath）角速度上限参数 `max_vel_theta`，评估其对导航结果与耗时分布的影响。该参数主要影响机器人局部路径跟踪过程中的转向能力，从而可能改变到达耗时与轨迹稳定性。

## 固定实验口径
- 地图：沿用 Week1 既定地图
- 起点/终点：沿用 `picture/week1_day6_ABCDpoint.png`，固定起点 → 固定目标点（与 baseline/A 保持一致）
- 每组次数：20 runs
- 评估方式：`eval_v2_action_recovery.py`（单次记录模式，基于 action 结果与耗时；可选解析 BT log 统计恢复行为）
- 成功判据/超时口径：沿用 Week4 Day1 既定定义

## 参数设置
- baseline：FollowPath.max_vel_theta = 1.0
- B 组（theta=0.6）：FollowPath.max_vel_theta = 0.6
- B’ 组（theta=0.4）：FollowPath.max_vel_theta = 0.4（用于敏感性补充对比）

参数修改与回滚（运行时动态设置）：
    ros2 param set /controller_server FollowPath.max_vel_theta 0.6
    ros2 param get /controller_server FollowPath.max_vel_theta

回滚（用于手动回起点阶段提高移动效率）：
    ros2 param set /controller_server FollowPath.max_vel_theta 1.0
    ros2 param get /controller_server FollowPath.max_vel_theta

## 数据输出
- baseline：results/week4_baseline_runs.csv
- B 组（theta=0.6）：results/week4_B_theta06_runs.csv
- B’ 组（theta=0.4）：results/week4_theta04_runs.csv

## 结果汇总（summarize_action_csv.py）
baseline（n=20）：
- success_rate = 1.000
- avg_time = 13.139 s
- median_time = 12.996 s
- min_time = 12.471 s
- max_time = 14.007 s

B 组：theta=0.6（n=20）：
- success_rate = 1.000
- avg_time = 13.039 s
- median_time = 12.761 s
- min_time = 12.361 s
- max_time = 14.303 s

B’ 组：theta=0.4（n=20）：
- success_rate = 1.000
- avg_time = 13.477 s
- median_time = 13.409 s
- min_time = 12.471 s
- max_time = 14.409 s

## 对照结论
1) theta=0.6 相比 baseline：平均耗时略降低，中位数改善更明显；但最慢样本尾部略变差。  
2) theta=0.4 相比 baseline：平均耗时与中位数均上升，整体到达更慢。  
3) 在当前场景下各组成功率均为 100%，差异主要体现在耗时分布与局部转向效率，而非成功率提升。

