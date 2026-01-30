# Week5 Day2 — Map1（turtlebot3_world）跑满 20 次评估：数据 + 复盘（可复现版本）

## 1) 今日目标
- 在 **Map1（官方 turtlebot3_world，最稳基线）**上跑满 **20 次**评估，拿到可对比的原始 CSV。
- 确认评估链路“真能用”：**脚本能自动记录 success/time，且 recovery 能统计出来（如果确实触发）**。
- 给 Day3/Day4 打底：后面换地图时，先不调参，只采样。

---

## 2) 今日产出文件（仓库里必须看得见）
- `results/week5_map1_runs.csv`（20 runs 原始记录，含表头）
- `docs/week5_day2_map1_20runs.md`（本文）

---

## 3) 运行口径（今天开始就“锁死”，后面不再改）
### 3.1 评估判据
- `timeout = 120s`（脚本超时）
- `result`：本脚本只会写 `S/T/F`
  - `S` = SUCCEEDED
  - `T` = CANCELED
  - `F` = ABORTED

> 本周为了对比统一，**先接受脚本口径**。碰撞/卡死的细分（C）放 Week6 再做工程化加分。

### 3.2 每次 run 的动作流程
- **每次只记录“去终点”这一趟**（脚本运行期间只发一次 Nav2 Goal）。
- 跑完后 `Ctrl+C` 结束脚本回到终端，再发“回起点”的 goal（不计入本次 run）。
- **不要求每次都用 2D Pose Estimate 重置**（你已经验证：频繁重置会让 AMCL 不稳定）。

### 3.3 Recovery 统计口径
- 脚本统计 recovery 不是靠你肉眼看到什么，而是靠 **BT 日志文件**：
  - 它会记录 `bt_start_line`（脚本启动时 BT log 的行数）
  - 结束后再取 `bt_end_line`（结束时 BT log 的行数）
  - 然后只在 `[bt_start_line, bt_end_line)` 这段窗口里用关键词统计 recovery

---

## 4) 一键启动（Map1 固定口径）

### Terminal A（Gazebo / Map1）
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Terminal B（Nav2 / Map1）
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

### Terminal C（BT 日志落盘：这是 recovery 统计的“证据源”）
> 建议每天开工先清空一次，避免不同地图/不同天混在一起。

    : > /tmp/week5_day2_map1_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5_day2_map1_bt.log

### Terminal D（每次 run 启动一次：先开脚本，再点 Nav2 Goal）
> 关键顺序：**先启动脚本，再在 RViz 点 Nav2 Goal**，否则脚本抓不到 goal 或抓不到本轮 BT 窗口。

    cd ~/ros2_nav2_portfolio
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week5_day2_map1_bt.log \
      --keywords configs/recovery_keywords.txt \
      --out results/week5_map1_runs.csv \
      --note "W5D2 Map1 timeout=120" \
      --timeout 120 \
      --status_topic /navigate_to_pose/_action/status

### 每一轮 run 的节拍（否则每次重设pose时AMCL位姿会紊乱）
1) Terminal D 启动脚本（看到“现在去 RViz 点一次 Nav2 Goal”）
2) RViz 点一次 Nav2 Goal（去终点）
3) 等脚本自动写入 CSV 并退出
4) RViz 发“回起点”Goal（不计入 run）
5) 下一轮重复

---

## 5) 结果汇总（Map1 20 runs）
### 5.1 原始数据位置
- `results/week5_map1_runs.csv`

### 5.2 总体统计（time_sec 单位：秒）
| 指标 | 数值 |
|---|---:|
| runs | 20 |
| 成功（S） | 20 / 20 = 100% |
| 平均耗时 mean | 28.868 |
| 中位数 median | 27.244 |
| 最小 min | 25.160 |
| 最大 max | 37.117 |
| 标准差 std | 3.752 |
| P90 | 35.564 |

### 5.3 Recovery 统计（本日样本）
| 指标 | 数值 |
|---|---:|
| recovery=Y 的次数 | 6 / 20 = 30% |
| recovery 发生的 run | 7, 9, 12, 15, 18, 20 |
| recovery_types | ClearLocalCostmap（本日只出现这一类） |
| recovery_hits | 每次都是 2 |

### 5.4 Recovery 对耗时的影响（本日观察结论）
| 分组 | 次数 | 平均耗时 | 耗时范围 |
|---|---:|---:|---|
| recovery=N | 14 | 26.743 | [25.160, 30.774] |
| recovery=Y | 6 | 33.827 | [31.025, 37.117] |

结论（用人话）：
- **只要触发 ClearLocalCostmap，这一趟通常会更慢**（本日平均多约 7 秒）。
- 本日所有 31s 以上的样本，基本都伴随 recovery（典型“恢复拖慢整体时间”的证据）。

---

## 6) 问题复盘（今天最关键：为什么一开始“看见 recovery”，CSV 还写 N？）

### 6.1 我遇到的现象（最直观）
- 导航过程里我能看到明显的恢复动作（比如清局部 costmap 的那一下）。
- 但 CSV 里连续多条都是：
  - `recovery=N`
  - `recovery_hits=0`
  - 而且 `bt_start_line == bt_end_line`（典型就是 `8175,8175` 这种）

### 6.2 一句话根因（别绕弯子）
**不是脚本“不会记 recovery”，而是脚本统计 recovery 的证据源（btlog 文件）根本没在变。**

脚本的逻辑：
- 它只在 `--btlog` 文件的“新增内容窗口”里找关键词。
- 如果文件没有新增行数，窗口就是空的 ⇒ recovery 必然统计不到。

### 6.3 我是怎么确认“证据源没变”的（证据链）
当时检查 btlog 文件：
- 文件行数固定不变（例如长期停在 8175 行）
- 修改时间停在早前
- 这就意味着：**导航发生了什么都不会写进这个文件**，脚本自然只能写 `N/0`。

### 6.4 解决方案（最小改动，立刻见效）
把 `/behavior_tree_log` 这个 topic **落盘到脚本读取的 btlog 文件里**（终端 C）：

    : > /tmp/week5_day2_map1_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5_day2_map1_bt.log

然后再跑评估脚本。

### 6.5 如何验证“真的解决了”（不用主观感受）
看 CSV 里两个字段就够了：
- 解决前：`bt_start_line == bt_end_line`（窗口为空）
- 解决后：`bt_end_line > bt_start_line`（窗口有内容），并在Rviz跑出recovery后出现：
  - `recovery=Y`
  - `recovery_types=ClearLocalCostmap`
  - `recovery_hits>0`

这就是“可复现 + 可验证”的闭环。

---

## 7) 今日结论
- Map1 作为基线场景，**20/20 成功**，评估链路稳定可用（脚本能记录成功与耗时）。
- Recovery 统计链路已打通：通过 BT 日志落盘，脚本能在 CSV 中记录 recovery 发生与类型。
- 本日数据明确显示：**发生 ClearLocalCostmap 的 run 会明显变慢**。

---
