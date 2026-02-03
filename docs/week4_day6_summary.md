# Week4 — Nav2 参数消融对照实验报告（A / B / A+B）

## 1. 目的与结论概览

本周目标是以“单变量/双变量对照”的方式评估两类参数对导航表现的影响，并形成可复现的数据证据链。

- A（Goal Tolerance，xy_goal_tolerance=0.4）在固定路线（起点→C点）下，平均耗时相对 baseline 明显下降（-1.013s）。
- B（角速度上限 max_vel_theta=0.6）对平均耗时影响很小（-0.100s），对该路线收益不明显。
- B'（max_vel_theta=0.4）平均耗时上升（+0.338s），符合“角速度能力受限导致转向耗时增加”的直觉。
- A+B 与 A+B' 的结果均显示：在本路线难度较低、成功率长期为 100% 的前提下，“组合参数”的主要可观测差异仍集中在耗时分布上；其中 A 的贡献更主要，B/B' 对时间的影响在该场景中呈弱效应或被 A 的效应部分覆盖。

说明：本周所有组别 success_rate 均为 1.000（20/20），因此对比重点放在 time_sec 的统计分布。

---

## 2. 实验设置（为可比性固定）

### 2.1 场景与路线固定
- 地图与固定点位参考：picture/week1_day6_ABCDpoint.png
- 起点：图中下方固定起点
- 终点：C 点（Week4 各组实验以同一终点为基准进行对比）

### 2.2 评估指标
对每次导航记录：
- result（S / T / F）
- time_sec（单次导航耗时）
- recovery（是否触发恢复及恢复类型统计；本周数据中均为 N/0）

汇总指标：
- runs
- success_rate
- avg_time
- median_time
- min_time
- max_time

### 2.3 数据采集方法
- 使用脚本：scripts/eval_v2_action_recovery.py
- 同步记录 BT log，用于将导航 action 与 BT 行范围关联：
    source /opt/ros/humble/setup.bash
    rm -f /tmp/week4_dayX_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/week4_dayX_bt.log
- 采集策略：单次记录、人工重复运行（每次在 RViz 发送一次 Nav2 Goal；回到起点后重复），避免仿真 Reset 导致定位漂移、local/global costmap 不一致等干扰因素。

---

## 3. 对照组与参数定义

### 3.1 Baseline（对照组）
- 数据文件：results/week4_baseline_runs.csv
- 参数：使用当前系统基线参数（未做本周 A/B 的改动）

### 3.2 A 组（Goal Tolerance）
- 数据文件：results/week4_A_runs.csv
- 变更点：controller_server / general_goal_checker / xy_goal_tolerance
- 直觉：放宽到达判定范围可能减少末端“贴点微调/徘徊”的时间消耗，从而降低 time_sec。

### 3.3 B 组（角速度上限，theta=0.6）
- 数据文件：results/week4_B_theta06_runs.csv
- 变更点：controller_server / FollowPath / max_vel_theta = 0.6
- 直觉：角速度上限降低会影响转向效率；但若路线本身转向需求不高或规划器给出的转向幅度较小，影响可能不明显。

### 3.4 B' 组（角速度上限，theta=0.4）
- 数据文件：results/week4_theta04_runs.csv
- 变更点：controller_server / FollowPath / max_vel_theta = 0.4
- 直觉：进一步收紧角速度上限，预计耗时上升更明显。

### 3.5 A+B 组（tol=0.4 + theta=0.6）
- 数据文件：results/week4_AplusB_tol04_theta06_runs.csv
- 变更点：A 与 B 同时生效。

### 3.6 A+B' 组（tol=0.4 + theta=0.4）
- 数据文件：results/week4_AplusB_tol04_theta04_runs.csv
- 变更点：A 与 B' 同时生效。

---

## 4. 汇总结果（20 runs/组）

基线（baseline）：
- runs=20
- success_rate=1.000
- avg_time=13.139
- median_time=12.996
- min_time=12.471
- max_time=14.007

A（xy_goal_tolerance=0.4）：
- runs=20
- success_rate=1.000
- avg_time=12.126（相对 baseline：-1.013）
- median_time=11.811（相对 baseline：-1.185）
- min_time=11.661
- max_time=13.980

B（max_vel_theta=0.6）：
- runs=20
- success_rate=1.000
- avg_time=13.039（相对 baseline：-0.100）
- median_time=12.761（相对 baseline：-0.235）
- min_time=12.361
- max_time=14.303

B'（max_vel_theta=0.4）：
- runs=20
- success_rate=1.000
- avg_time=13.477（相对 baseline：+0.338）
- median_time=13.409（相对 baseline：+0.413）
- min_time=12.471
- max_time=14.409

A+B（tol=0.4 + theta=0.6）：
- runs=20
- success_rate=1.000
- avg_time=12.365（相对 baseline：-0.774）
- median_time=12.061（相对 baseline：-0.935）
- min_time=11.661
- max_time=14.248

A+B'（tol=0.4 + theta=0.4）：
- runs=20
- success_rate=1.000
- avg_time=12.494（相对 baseline：-0.645）
- median_time=12.261（相对 baseline：-0.735）
- min_time=11.860
- max_time=13.653

---

## 5. 结果解读（面向工程口径）

### 5.1 为什么 success_rate 全为 1.000 仍然有意义
该路线在当前地图与代价地图配置下难度较低，系统已稳定达到 100% 成功率。此时参数优化的可观测收益更可能体现在：
- time_sec 的整体下移（效率）
- time_sec 的尾部收敛（稳定性）
- recovery 触发减少（本周该路线未触发，因此无法用于区分）

因此，本周对比重点采用耗时统计而非成功率。

### 5.2 A 组对耗时下降的解释
放宽 xy_goal_tolerance 会降低“末端精确贴合目标位姿”的需求，减少末端阶段的小幅反复修正，从而使 time_sec 下降。该效应在 A 组中表现为：
- avg_time 与 median_time 均明显下降（约 1 秒量级）
- 最小值进一步降低（min_time=11.661）

### 5.3 B/B' 组对耗时的解释
角速度上限的变化会影响转向动作的完成速度：
- theta=0.6 相对 baseline 的变化幅度有限，且该路线的转向需求可能不足以放大差异，因此统计上呈弱效应（-0.100s）。
- theta=0.4 对转向能力限制更明显，导致 avg_time 与 median_time 上升，符合预期。

### 5.4 A+B 与 A+B' “抵消/覆盖”的工程解释
在该路线中：
- A 的主要贡献来自“终点判定更宽松”带来的末端耗时下降；
- B/B' 的贡献来自“转向速度上限”对路径中转向段耗时的影响。

当两者组合时，整体耗时会同时受到两类机制影响，可能出现：
- A 的下降效应覆盖了 B' 的上升效应（从而总体仍比 baseline 更快）；
- B 的弱效应在 A 的强效应下难以分离（表现为 A+B 与 A 结果接近）。

该现象并不矛盾，反映的是：在当前任务难度下，末端判定条件对 time_sec 的影响权重更大。

---

## 6. 回滚与复现实用口径

### 6.1 参数在线修改与回滚
以 controller_server 为例：

- 修改：
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.4
    ros2 param set /controller_server FollowPath.max_vel_theta 0.6

- 回滚到常用值（示例）：
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.25
    ros2 param set /controller_server FollowPath.max_vel_theta 1.0

- 查询确认：
    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
    ros2 param get /controller_server FollowPath.max_vel_theta

---

## 7. 限制与后续工作（可选增强）

- 当前路线难度偏低，success_rate 长期饱和，导致“成功率提升”难以作为区分指标。
- 若后续需要进一步区分参数效果，可增加更具挑战的对照条件：
  - 更远的目标点（如从 C 切换到 D，或增加转弯/窄通道段）
  - 引入障碍扰动或动态障碍
  - 记录额外指标（路径长度、角速度峰值、控制抖动次数、恢复行为类型分布等）
