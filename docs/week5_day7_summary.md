# Week5 Day7 — 本周总结（Map 泛化 + Bag Record/Replay 证据链）

> 这一周我只干了两件事：  
> 1）用三张地图把“泛化压力”测出来；  
> 2）把 bag 录制/回放打通，并且把 recovery 的证据链闭环（不再靠“我看见了”这种主观说法）。

---

## 0. 一句话结论（给面试官看的）
- **Map1 是稳定基准**（20/20 成功，平均 28.87s，recovery 很少）。  
- **Map2 开始有压力**（成功率 0.75，平均 75.14s，recovery 次数显著上升）。  
- **Map3 基本跑不动**（成功率 0.05，绝大多数 timeout，recovery 几乎全程在自救）。  
- **Bag replay 的价值不在“更快”，在“输入固定、可复现、便于做 A/B 对照与复盘定位问题”。**

---

## 1. 本周文件总览（我已经落库到仓库，不需要去翻前几天）
### 1.1 我现在仓库里的关键文件（真实 ls 输出）
    cd ~/ros2_nav2_portfolio
    ls -lh results/week5_* results/week5d6_* 2>/dev/null
    ls -lt bags | head

我当前看到的结果（口径以这份为准）：
- results/week5_bag_smoke_runs.csv
- results/week5_map1_runs.csv
- results/week5_map2_runs.csv
- results/week5_map3_runs.csv
- results/week5_summary_table.csv
- results/week5d6_map1_live_run01.csv
- results/week5d6_map1_bag_replay_run01.csv
- results/week5d6_map1_replay_steady10_run01.csv

bags/：
- bags/week5d6_map1_batch01
- bags/week5_map1_run01

---

## 2. 本周做了哪几件事（按天归档）
### Day1：三张地图 + 固定协议
- Map1：系统自带地图（前几周一直用，稳定基准）
- Map2：自画 mid_house（中等难度）
- Map3：自画 narrow_house（高难度）
- 固定每张地图跑 20 次，写清楚成功/超时/失败口径

### Day2：Map1 20 runs（稳定基准）
产物：results/week5_map1_runs.csv

### Day3：Map2 + Map3 20 runs（泛化压力）
产物：
- results/week5_map2_runs.csv
- results/week5_map3_runs.csv

### Day4：三张地图泛化汇总（我已经汇总到一张表）
产物：results/week5_summary_table.csv

### Day5：Bag 录制/回放跑通（烟雾测试）
产物：results/week5_bag_smoke_runs.csv + docs 里对应说明

### Day6：Map1 做 Live vs Replay 对照（重点：证据链闭环）
产物：
- bags/week5d6_map1_batch01（bag 数据集）
- results/week5d6_map1_live_run01.csv（Live 10 次）
- results/week5d6_map1_bag_replay_run01.csv（Replay 原始 runs）
- results/week5d6_map1_replay_steady10_run01.csv（Replay 稳态 10 次，用于对照）

---

## 3. Day4 的泛化结果（Week5 核心成果之一）
以下来自 results/week5_summary_table.csv（每张地图 20 runs）：

| Map | Runs | S | T | F | SuccessRate | MeanTime(s) | MeanRecoveryHits |
|---|---:|---:|---:|---:|---:|---:|---:|
| Map1 | 20 | 20 | 0 | 0 | 1.00 | 28.8679 | 0.60 |
| Map2 | 20 | 15 | 4 | 1 | 0.75 | 75.1373 | 5.85 |
| Map3 | 20 | 1 | 19 | 0 | 0.05 | 117.3593 | 15.55 |

补充：Map3 的 timeout 里面，很多是“视觉上已接近目标但未触发 reached”（visual_reached_rate=0.8421）。

我自己的结论（不绕弯）：
- **Map1 能当 baseline**：稳定、耗时低、恢复少。
- **Map2 是真实泛化压力**：能跑，但恢复动作明显增多，时间被拉长。
- **Map3 是极限难度**：几乎一直在自救（clear costmap / spin / backup），多数超时。

---

## 4. Bag Replay 的意义（我用一句话讲清楚）
Bag replay 不是为了“更快”，是为了：
- **把输入（/tf /scan /odom /map /clock /behavior_tree_log）固定成数据集**；
- 然后让 Nav2 在同一份输入上反复跑，方便：
  - 复现问题（不是靠运气）
  - 做参数 A/B（输入一致，结论更硬）
  - 复盘 recovery 发生的位置与次数（证据可查）

一句比喻：**录像回放**。回放时“机器人自己在跑”很正常，因为 TF/里程计/scan 都在按 bag 时间轴被重放。

---

## 5. Live vs Replay 对照（Week5 核心成果之二）
### 5.1 Live（10 次，Map1 goal=D）
文件：results/week5d6_map1_live_run01.csv  
现象：耗时波动较大，recovery 偶发。

### 5.2 Replay（稳态 10 次，Map1 goal=D）
文件：results/week5d6_map1_replay_steady10_run01.csv  
现象：耗时更稳定，recovery=0（至少在我这 10 次稳态区间里是这样）。

### 5.3 我对“Replay 时间与 Live 接近”的解释（只写合理怀疑，不硬吹）
可能原因之一：
- 刚启动链路时 TF/scan/LiDAR 等存在“预热/收敛”，即使我做过缓冲，也很难每次精确扣掉这部分启动延迟；
- replay 在理想条件下**可能**更快，但本周数据更支持的结论是：**更稳定/更可复现**，不强行下“必然更快”的结论。

---

## 6）本周踩坑复盘（问题是什么 + 我怎么修到“可复现”）

> 说明：这是 **本周问题概括版**。完整命令/截图/细节请直接看：`docs/week5_day5_map1_bag_replay_setup.md`。

---

### 6.1 `/clock` Publisher count 变成 2（最容易把后面全带崩）
**现象**
- `ros2 topic info -v /clock` 显示 Publisher count: 2

**影响**
- 时间源不干净：TF/RViz 会“闪/抖/对不上时间”，后面所有排查都在浪费。

**解决（硬口径）**
- 回放前必须清场（`pkill`），保证只启动一次回放链路。
- 回放时间源只选一种：  
  - **要么**播 bag 自带 `/clock`（推荐）  
  - **要么**用 `--clock` 注入  
  - **禁止混用**（混用最容易直接 Publisher=2）

**验证**
- Terminal B 里 `/clock` 发布者必须 = 1，才继续下一步。

---

### 6.2 `/map` “No map received” 或“出来一下又没了”
这是今天最烦的坑，根因主要两类：

#### 6.2.1 QoS 不匹配：你错过 bag 里那 1 条 `/map`
**证据**
- `ros2 bag info ...`：`Topic: /map ... Count: 1`（bag 里 /map 只有 1 条）
- 发布端多为 `Durability: TRANSIENT_LOCAL`
- RViz 的 Map Display 订阅若是 `VOLATILE`，对不上就会“错过一次就永远没”。

**解决**
- RViz → Map Display：把 QoS **Durability Policy = Transient Local**
- 同时建议：**取消勾选 Use Timestamp**（回放里 TF/时间戳更容易互相搞你）

**验证**
- `ros2 topic info -v /map`：Publisher count=1
- RViz Map 能稳定显示（不是一闪就没）。

#### 6.2.2 `--loop` 时间回拨导致 RViz “闪/丢显示”（看起来像 map 消失）
**现象**
- Map 出来一会儿又没了；Ctrl+C 再播又闪一下。

**根因**
- `--loop` 让 `/clock` 回到起点，RViz 的 TF buffer / 缓存可能被时间回拨搞乱。

**解决**
- 排查阶段默认 **不 loop**，先把链路稳定跑通。
- 真要 loop：接受它可能闪；最稳做法是“loop + 需要时重启 RViz”，别指望它永远不抖。

---

### 6.3 RobotModel 红 / TF 闪：`/robot_description` 里残留 `${namespace}`
**现象**
- RViz RobotModel 报错、不显示（红）
- 机器人相关 frame 看起来像 TF 断链
- 关键证据：`/robot_description` 里能 grep 到 `${namespace}` 这类占位符

**一句话根因**
- `/robot_description` 发布的是带变量的模板文本，不是最终 URDF；RViz 不会替你展开变量，直接解析失败。

**解决（我最终生效的做法）**
- 我本机 `turtlebot3_description` 没有 `.urdf.xacro`，所以不走 xacro 路线。
- 改为：**直接用本机存在的 `.urdf` 发布 `/robot_description`**  
- QoS 必须 `TRANSIENT_LOCAL`（保证 RViz 后开也能收到）

**验证（硬口径）**
- `grep -oE '\$\{[^}]+\}'` 的数量必须为 0（看不到 `${...}` 才算彻底修好）。

---

### 6.4 Map1 的 yaml 要不要加载（我给自己的口径）
- **只做回放可视化/复盘**：不需要加载 yaml；bag 里本来就有 `/map`（但注意可能只有 1 条，必须配合 RViz QoS）
- **想让 Map 永远稳定、别吃“/map 只有 1 条”的亏**：不要回放 `/map`，改为运行 `map_server` 用 Map1 的 yaml 来发布 `/map`（工程化更稳，但属于另一条链路）

---

## 7. 本周实验的“自包含启动方式”（不依赖前几天文档）

> 注意：eval_v2 脚本一次 run 结束会自动退出，所以 Terminal D 每次都要重启一次（这点我已经接受，反正保证口径一致）。

### 7.1 Live（Gazebo 真跑 + eval 记 CSV，可选同时录 bag）
Terminal A（Gazebo）：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal B（Nav2）：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

Terminal C（BT log 落盘：recovery 统计证据源）：
    : > /tmp/week5d6_map1_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5d6_map1_bt.log

Terminal D（评估脚本：一次 run 结束自动退出）：
    cd ~/ros2_nav2_portfolio
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week5d6_map1_bt.log \
      --timeout 120 \
      --out results/week5d6_map1_live_run01.csv \
      --note "W5D6 map1 goal=D live run01"

Terminal E（可选：bag 录制一批 runs）：
    cd ~/ros2_nav2_portfolio
    ros2 bag record -o bags/week5d6_map1_batch01 \
      /clock /tf /tf_static /scan /odom /map /cmd_vel /behavior_tree_log

Live 单次 run 的人工操作：
- RViz：点 2D Pose Estimate 设置初始位姿
- RViz：点 Nav2 Goal 设置目标点 D
- 等 reached/timeout
- Terminal D 自动退出后，重复下一次 run（重启 Terminal D）

---

### 7.2 Replay（bag 回放 + Nav2 吃回放数据 + eval 记 CSV）
Terminal A（bag 回放）：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    cd ~/ros2_nav2_portfolio
    ros2 bag play bags/week5d6_map1_batch01 --loop \
      --topics /clock /tf /tf_static /scan /map /odom /behavior_tree_log

Terminal B（Nav2）：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

Terminal C（BT log 落盘：同样必须开，否则 recovery 统计为 0）：
    : > /tmp/week5d6_replay_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5d6_replay_bt.log

Terminal D（评估脚本）：
    cd ~/ros2_nav2_portfolio
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week5d6_replay_bt.log \
      --timeout 120 \
      --out results/week5d6_map1_bag_replay_run01.csv \
      --note "W5D6 replay map1 goal=D run01"

Replay 单次 run 的人工操作：
- RViz：点 2D Pose Estimate（把定位“认为的起点”设回去）
- RViz：点 Nav2 Goal（目标点 D）
- Terminal D 自动退出后，重复下一次 run（重启 Terminal D）

说明（我自己踩过坑）：
- 回放时看到机器人“自己在跑”正常：TF/里程计/scan 都来自 bag。
- replay 不是 Gazebo 物理世界，严格说不能“拖回起点”，我能做的是在 RViz 里把初始位姿重新设回起点。

---

## 8. 我对 Week6 的计划（承上启下）
Week6 我不会再“凭感觉跑两次就下结论”。  
我会优先用 Replay 固化输入，然后做参数 A/B/A+B（2×2）对照；  
结论跑出来后，再回到 Live 做抽检验证，确认不是“只在回放里成立”。

