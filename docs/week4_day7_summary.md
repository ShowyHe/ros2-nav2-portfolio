# Week4 Day6 — Week4 总结（可一键复现）

> 本周目标：用**同一张地图 + 固定起点/终点**，对比 6 组配置（Baseline / A / B(theta=0.6) / B'(theta=0.4) / A+B(theta=0.6) / A+B'(theta=0.4)），输出**可复现的汇总表**与**可解释的结论**。  
> 说明：Week3 Day1–Day2 使用 `scripts/eval_v1_action_logger.py`；从 Week3 Day3 起（含 Week4 全部实验）统一使用 `scripts/eval_v2_action_recovery.py`。统计脚本是 `scripts/summarize_action_csv.py`。

---

## 0. 本周数据与脚本对照

### 结果文件（results/）
- `results/week4_baseline_runs.csv`
- `results/week4_A_runs.csv`
- `results/week4_B_theta06_runs.csv`
- `results/week4_theta04_runs.csv`
- `results/week4_AplusB_tol04_theta06_runs.csv`
- `results/week4_AplusB_tol04_theta04_runs.csv`
- `results/week4_summary_table.csv`

### 脚本（scripts/）
- Week3 Day1–Day2：`scripts/eval_v1_action_logger.py`（action status 自动计时）
- Week3 Day3 起（含 Week4）：`scripts/eval_v2_action_recovery.py`（action + recovery + btlog 行号）
- 统计汇总：`scripts/summarize_action_csv.py`（输出 runs/avg/median/min/max 等）

---

## 1. 实验固定口径（保证可比）

- 场景：TB3 Gazebo + Nav2（同一地图/同一 RViz 配置）
- 起点/终点：**固定**（Week4 口径：起点固定、终点固定为 C 点；若后续换 D 点必须另开一套对照数据，不与本周汇总混算）
- 每次 run 的执行方式（手动可控版）：
  1) 机器人处于起点
  2) 终端 D 启动一次 `eval_v2_action_recovery.py`（它会等待一次 goal）
  3) RViz 点一次 `Nav2 Goal`（起点→终点）
  4) 脚本写入 CSV 后自动退出
  5) 人工回到起点（此过程**不计入 run**）

---

## 2. 一键复现：通用启动（Gazebo + Nav2 + RViz）

### Terminal A — Gazebo
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Terminal B — Nav2
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

### RViz 必做
- Fixed Frame = `map`
- 先点 `2D Pose Estimate` 设置初始位姿
- 再点 `Nav2 Goal` 下发目标点

---

## 3. 一键复现：BT log 记录（Terminal C）

> 目的：让 `eval_v2_action_recovery.py` 能写入 `bt_start_line/bt_end_line`，用于定位是否触发恢复与恢复类型。

### 每天/每个实验开始前（重开一个干净的 btlog）
    source /opt/ros/humble/setup.bash
    rm -f /tmp/week4_dayX_bt.log
    ros2 topic echo /behavior_tree_log > /tmp/week4_dayX_bt.log

自检（确认文件在增长）：
    ls -lh /tmp/week4_dayX_bt.log
    wc -l /tmp/week4_dayX_bt.log
    tail -n 3 /tmp/week4_dayX_bt.log

> 注：`/tmp/week4_dayX_bt.log` 在 Linux 的 `/tmp/` 目录（临时目录）。  
> 不建议提交到 Git（体积大、内容噪声多），但可以保留本地作为证据链。

---

## 4. 参数实验（Week4 的 A/B/A+B）与回滚命令（可直接复制）

> 重要：你是“直接改运行时参数 + 跑完立刻改回去”的模式。下面给出 **get → set → 再 get 验证 → 回滚** 的最短闭环命令。

### 4.1 实验 A：Goal tolerance（xy_goal_tolerance）
目标：比较“到点判定更宽松/更严格”是否显著影响 time_sec / recovery

查看当前值：
    source /opt/ros/humble/setup.bash
    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

设置为 A（示例：0.4）并验证：
    source /opt/ros/humble/setup.bash
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.4
    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

回滚为 baseline（示例：0.25）并验证：
    source /opt/ros/humble/setup.bash
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.25
    ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

---

### 4.2 实验 B：角速度上限（FollowPath.max_vel_theta）
目标：比较“更快转向/更慢转向”对 time_sec 的影响（以及是否触发 recovery）

查看当前值：
    source /opt/ros/humble/setup.bash
    ros2 param get /controller_server FollowPath.max_vel_theta

设置 B（theta=0.6）并验证：
    source /opt/ros/humble/setup.bash
    ros2 param set /controller_server FollowPath.max_vel_theta 0.6
    ros2 param get /controller_server FollowPath.max_vel_theta

设置 B'（theta=0.4）并验证：
    source /opt/ros/humble/setup.bash
    ros2 param set /controller_server FollowPath.max_vel_theta 0.4
    ros2 param get /controller_server FollowPath.max_vel_theta

回滚为 baseline（theta=1.0）并验证：
    source /opt/ros/humble/setup.bash
    ros2 param set /controller_server FollowPath.max_vel_theta 1.0
    ros2 param get /controller_server FollowPath.max_vel_theta

---

## 5. 一键复现：单次 run 的记录（Terminal D）

> 每次 run：先启动脚本（等待 goal）→ RViz 点一次 Nav2 Goal → 脚本写入 CSV 自动退出。

通用模板：
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_dayX_bt.log \
      --out results/<your_csv>.csv \
      --note "<your_note>"

### 5.1 Baseline（20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day2_bt.log \
      --out results/week4_baseline_runs.csv \
      --note "W4 baseline"

### 5.2 A（tol=0.4，20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day3_bt.log \
      --out results/week4_A_runs.csv \
      --note "W4 A tol=0.4"

### 5.3 B（theta=0.6，20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day4_theta06_bt.log \
      --out results/week4_B_theta06_runs.csv \
      --note "W4 B theta=0.6"

### 5.4 B'（theta=0.4，20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day4_theta04_bt.log \
      --out results/week4_theta04_runs.csv \
      --note "W4 Bprime theta=0.4"

### 5.5 A+B（tol=0.4 + theta=0.6，20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day5_ab_tol04_theta06_bt.log \
      --out results/week4_AplusB_tol04_theta06_runs.csv \
      --note "W4 AplusB tol=0.4 theta=0.6"

### 5.6 A+B'（tol=0.4 + theta=0.4，20 runs）
    cd ~/ros2_nav2_portfolio
    source /opt/ros/humble/setup.bash
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week4_day5_ab_tol04_theta04_bt.log \
      --out results/week4_AplusB_tol04_theta04_runs.csv \
      --note "W4 AplusB tol=0.4 theta=0.4"

---

## 6. 一键复现：统计汇总（生成 week4_summary_table.csv）

逐个输出统计（终端直接看）：
    cd ~/ros2_nav2_portfolio
    python3 scripts/summarize_action_csv.py --in results/week4_baseline_runs.csv
    python3 scripts/summarize_action_csv.py --in results/week4_A_runs.csv
    python3 scripts/summarize_action_csv.py --in results/week4_B_theta06_runs.csv
    python3 scripts/summarize_action_csv.py --in results/week4_theta04_runs.csv
    python3 scripts/summarize_action_csv.py --in results/week4_AplusB_tol04_theta06_runs.csv
    python3 scripts/summarize_action_csv.py --in results/week4_AplusB_tol04_theta04_runs.csv

本周汇总表（已得到的统计结果）：
- baseline:
  - runs=20, success_rate=1.000, avg=13.139, median=12.996, min=12.471, max=14.007
- A (tol=0.4):
  - runs=20, success_rate=1.000, avg=12.126, median=11.811, min=11.661, max=13.980
- B (theta=0.6):
  - runs=20, success_rate=1.000, avg=13.039, median=12.761, min=12.361, max=14.303
- B' (theta=0.4):
  - runs=20, success_rate=1.000, avg=13.477, median=13.409, min=12.471, max=14.409
- A+B (tol=0.4, theta=0.6):
  - runs=20, success_rate=1.000, avg=12.365, median=12.061, min=11.661, max=14.248
- A+B' (tol=0.4, theta=0.4):
  - runs=20, success_rate=1.000, avg=12.494, median=12.261, min=11.860, max=13.653

`results/week4_summary_table.csv` 表头建议：
    group,file,runs,success_rate,avg_time,median_time,min_time,max_time

（如：`results/week4_summary_table.csv`）

---

## 7. Week4 结论（写给面试官的版本）

### 7.1 现象总结（只说数据）
- 所有组均 20/20 成功，recovery=0（在当前路线与地图下“任务难度较低”，成功率指标失去区分度）。
- 最明显的变化来自 A（tol=0.4）：avg_time 约从 13.139 降到 12.126（更容易判定到达，整体耗时下降）。
- B（theta=0.6）对耗时影响很小；B'（theta=0.4）平均耗时上升（转向上限过低导致局部机动性下降）。
- A+B（tol=0.4 + theta=0.6）保持较快；A+B'（tol=0.4 + theta=0.4）相对变慢但仍比 baseline 快，说明 **A 的“更宽松到达判定”在一定程度上抵消了 B' 的“转向变慢”**。

### 7.2 为什么会出现 “A+B 看起来抵消/不明显”
- A（tolerance）影响的是**终点判定**：更宽松 → 更早 SUCCEEDED → time_sec 变短（尤其在终点附近“绕/修正”的阶段）。
- B（theta 上限）影响的是**轨迹跟踪机动性**：更小 → 转向更慢 → 在弯折/对齐阶段耗时可能增加。
- 当路径本身非常简单（起点→C 点较容易），theta 对整体耗时的贡献被稀释；而 tolerance 会直接截断末端对齐时间，因此 A 的效应更显著。

### 7.3 本周不足（可作为“下一步计划”）
- 当前地图与目标点过于容易：success_rate 和 recovery 指标无法拉开差距。
- 下一步（Week5/Week6 可扩展）建议：
  - 换更难的终点（例如 D 点或更贴障碍的位置）并重新建立 baseline；
  - 引入障碍干扰或更严格 time_allowance，让失败/恢复更可能出现，从而比较更有信息量。

---

## 8. 提交清单（本周你应该推到 GitHub 的）
- docs：
  - `docs/week4_day1_experiment_design.md`
  - `docs/week4_day2_baseline.md`
  - `docs/week4_day3_A_goal_tolerance.md`
  - `docs/week4_day4_B_max_vel_theta.md`
  - `docs/week4_day5_AplusB.md`
  - `docs/week4_day6_summary.md`（本文）
- results：
  - 6 个 runs CSV（见第 0 节）
  - `results/week4_summary_table.csv`
- scripts：
  - `scripts/eval_v2_action_recovery.py`
  - `scripts/summarize_action_csv.py`

--- 

## 9. 常见踩坑（复现时最容易出错的 3 件事）
1) Terminal C 的 btlog 没在增长 → `bt_start_line/bt_end_line` 可能全是 0 或不可信  
2) 参数 set 了但没 get 验证 → 实际跑的可能还是旧值  
3) 脚本启动顺序错了（先点 goal 后开脚本）→ 这次 run 不会被记录进 CSV

