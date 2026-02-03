# Week5 Day6 — Bag Replay vs Live 对比（Map1 / Goal=D）

## 0) 今日目的
把 **bag 回放（replay）** 和 **实时仿真（live）** 的行为差异用数据说清楚：**成功率/耗时/恢复（recovery）** 到底差在哪，为什么差。

---

## 1) 实验口径（写死，避免扯皮）
- 地图：Map1
- 目标点：D 点（详见week5_map1_Epoint.png）
- timeout：120s（脚本参数）
- 成功判据：以 action 结果为准（CSV 里 result=S/F/T）
- 恢复统计证据源：`/behavior_tree_log` 落盘到 `--btlog` 文件，由 `eval_v2_action_recovery.py` 在新增窗口内统计

> 说明：recovery 不是“我肉眼看到就算”，而是 **BT log 落盘后脚本统计到的**，否则 CSV 会出现 recovery=N 的假阴性。

---

## 2) Live（实时仿真）完整启动方式（A/B/C/D）
目标：跑 10 次到 D 点，并产出 live CSV + BT 落盘日志。

### Terminal A（Gazebo）
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    export ROS_DOMAIN_ID=88
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Terminal B（Nav2）
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    export ROS_DOMAIN_ID=88
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

### Terminal C（BT log 落盘，必须常开）
    : > /tmp/week5d6_map1_live_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5d6_map1_live_bt.log

### Terminal D（评估脚本：每次 run 都重启一次，并手动让机器人跑回到终点）
说明：脚本默认“跑完一次 goal 就退出”，所以每次 run 都重新执行一次。

    cd ~/ros2_nav2_portfolio
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week5d6_map1_live_bt.log \
      --timeout 120 \
      --out results/week5d6_map1_live10.csv \
      --note "W5D6 live map1 goal=D"

执行顺序（每一轮 run）：
- 先启动 Terminal D（脚本打印“现在去 RViz 点一次 Nav2 Goal”）
- 再去 RViz 点一次 Nav2 Goal（目标 D）
- 等脚本打印 [logged] 并退出
- 下一轮 run：再重启 Terminal D，再点一次 Nav2 Goal

> 注意：live 不建议频繁 reset/瞬移。若必须瞬移回起点，至少要：
> - RViz 重新 2D Pose Estimate（重设初始位姿）
> - clear local/global costmap
> - 等 AMCL 收敛（粒子云集中/pose 稳定）后再开始下一次 run  
> 否则会引入“定位自救时间”，污染耗时与 recovery 统计。

---

## 3) Replay（bag 回放）完整启动方式（A/B/C/D）
目标：对同一个 bag 进行回放，重复 10 次到 D 点，并产出 replay CSV + BT 落盘日志。

### 3.1 Terminal A（bag play 回放）
说明：本次使用的 bag 目录示例为 `bags/week5d6_map1_batch01`（以你的实际路径为准）。
回放建议只回放必要 topics（避免回放多余内容干扰/浪费资源）。

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    cd ~/ros2_nav2_portfolio
    ros2 bag play bags/week5d6_map1_batch01 --loop \
      --topics /clock /tf /tf_static /scan /map /odom /behavior_tree_log

关键点：
- 回放里 **/clock 必须只有一个发布者**（bag 在发 /clock，别再加 --clock 造成双钟）
- Nav2 / RViz 必须 use_sim_time:=True

### 3.2 Terminal B（Nav2，use_sim_time 必开）
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    export ROS_DOMAIN_ID=88
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

> 说明：replay 模式下不需要 Gazebo；Nav2 用回放数据驱动即可。

### 3.3 Terminal C（BT log 落盘，必须常开）
回放时 `/behavior_tree_log` 是从 bag 里出来的，但我依然把它落到文件里，确保脚本统计 recovery 有证据源。

    : > /tmp/week5d6_map1_replay_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5d6_map1_replay_bt.log

### 3.4 Terminal D（评估脚本：每次 run 都重启一次）
    cd ~/ros2_nav2_portfolio
    python3 scripts/eval_v2_action_recovery.py \
      --btlog /tmp/week5d6_map1_replay_bt.log \
      --timeout 120 \
      --out results/week5d6_map1_replay_steady10.csv \
      --note "W5D6 replay map1 goal=D (steady10)"

执行顺序（每一轮 run）：
- 回放启动并稳定（建议等 10 秒，确保 /tf /scan 在流）
- 启动 Terminal D（脚本提示你去 RViz 点 goal）
- 去 RViz 点一次 Nav2 Goal（目标 D）
- 等脚本打印 [logged] 并退出
- 下一轮 run：再重启 Terminal D，再点一次 Nav2 Goal

关于“机器人怎么回到起点”：
- replay 不是 Gazebo，没有“拖动回起点”这回事
- 每次 run 的起点取决于：在 RViz 里重新设定的初始位姿（2D Pose Estimate）以及定位收敛情况
- 如果发现“点 goal 后要等很久才开始走”，优先怀疑：
  - TF/scan 还没稳定（回放刚启动）
  - costmap 还在初始化/清理
  - 初始 pose 设定后还没收敛（粒子云没集中）

---

## 4) 数据来源（两组必须明确区分）
### 4.1 Live（实时仿真）— 10 runs
- 含义：Gazebo 实时跑，Nav2 实时算，传感器与时钟实时流动
- 数据：10 runs（全部成功）

### 4.2 Replay（bag 回放）— 稳态 10 runs
- 含义：用 ros2 bag play 回放输入，Nav2 在回放输入上运行
- 关键点：回放存在“冷启动偏置”（前几次 costmap/TF/scan 还没热起来，容易触发开局 recovery）
- 本次做法：我先跑出了 15 次 replay，详见文件week5d6_map1_bag_replay_run01.csv，其中前 5 次是明显冷启动恢复偏置；因此本对比只采用 **稳态 10 次（原 replay 的 runs 6–15）** 详见文件week5d6_map1_replay_steady10_run01.csv，作为 replay 对比样本

> 这一步不是“耍赖删数据”，而是为了公平：我比较的是“导航行为”，不是比较“系统刚启动那 20 秒的初始化抖动”。

---

## 5) 原始 CSV（用于对比的两份数据）
### 5.1 Live 10 runs
run,goal_id,result,time_sec,notes,recovery,recovery_types,recovery_hits,bt_start_line,bt_end_line
1,9042104bd8e51c2269d0530a4b51aad6,S,27.928,W5D6 map1 goal=D run=01,N,,0,0,1727
2,5ff0eaae5e080ce066c5aac96091bf5d,S,40.131,W5D6 map1 goal=D run=01,Y,ClearLocalCostmap,2,1727,4177
3,cce033f2549b639cd12c81754a77a5a1,S,43.716,W5D6 map1 goal=D run=01,Y,ClearLocalCostmap,2,4177,6947
4,472b2b9e2d9c93f2d9c93f2a8a7f2f8a1be31bb,S,30.906,W5D6 map1 goal=D run=01,N,,0,6947,8863
5,55a7c0641f99aaf2c22c4242b3682067,S,26.820,W5D6 map1 goal=D run=01,N,,0,8863,10527
6,e2cdb032a5f1977ce0cff39075c4e65c,S,29.109,W5D6 map1 goal=D run=01,N,,0,10527,12317
7,4f78a303a8c87d13bbacde69a6b4c5b4,S,27.346,W5D6 map1 goal=D run=01,N,,0,12317,13981
8,d4efb09888cbda596e3db081fc010a46,S,32.672,W5D6 map1 goal=D run=01,Y,ClearLocalCostmap,2,13981,16058
9,011d9ad0caea5e0d771bc2199abf9280,S,26.870,W5D6 map1 goal=D run=01,N,,0,16058,17848
10,3e9ad930b28a3b0e11d0ecc35186ef24,S,27.006,W5D6 map1 goal=D run=01,N,,0,17848,19449

### 5.2 Replay（稳态 10 runs）
run,goal_id,result,time_sec,notes,recovery,recovery_types,recovery_hits,bt_start_line,bt_end_line
1,c074e020aaf67a230140a2d84ad116d2,S,34.584,W5D6 replay map1 goal=D run=01,N,,0,48552,56216
2,f2f6d46c4091b90fb48b9dfac0ecc08b,S,31.023,W5D6 replay map1 goal=D run=01,N,,0,56216,63376
3,6e17dda63c43dcf1571a74bf9885ad52,S,31.829,W5D6 replay map1 goal=D run=01,N,,0,63376,70662
4,39521815179a570c8654725be9a0fe6a,S,32.484,W5D6 replay map1 goal=D run=01,N,,0,70662,77948
5,99c6016a16cd596f9678a640dc557e3c,S,29.656,W5D6 replay map1 goal=D run=01,N,,0,77948,84982
6,21f503c1a2f1844a4c0a581d915e12f5,S,35.808,W5D6 replay map1 goal=D run=01,N,,0,84982,92772
7,6e8ba430bd581d73d22995f50a6cc6bb,S,32.588,W5D6 replay map1 goal=D run=01,N,,0,92772,100184
8,1d75bc21dab5b4dccf5abb6c187a1bce,S,31.092,W5D6 replay map1 goal=D run=01,N,,0,100184,107344
9,a6a04f9c4c3cbefb5cd520e3b2dd3fb3,S,30.341,W5D6 replay map1 goal=D run=01,N,,0,107344,114378
10,b9c697f1be41a22b27f590dcf2072bed,S,30.504,W5D6 replay map1 goal=D run=01,N,,0,114378,121528

---

## 6) 对比结果（核心表）
| mode  | N  | success_rate | mean_time_sec | median_time_sec | recovery_rate | recovery_types |
|------|----|--------------|---------------|-----------------|---------------|----------------|
| live | 10 | 100%         | 31.25         | 28.52           | 30% (3/10)    | ClearLocalCostmap |
| replay (steady) | 10 | 100% | 31.59 | 31.46 | 0% (0/10) | - |

> 注：mean/median 为我按 CSV 直接计算得到（后续可用 summarize_action_csv.py 再验一次，口径一致即可）。

---

## 7) 现象解读
### 7.1 成功率：live vs replay 一致
两组都是 10/10 成功，说明：
- 这张图从起点到 D 点在当前参数下 **可达性没问题**
- 主要差异不在“能不能到”，而在“过程是否需要恢复、耗时分布是否稳定”

### 7.2 recovery：live 有，replay（稳态）没有
- live：3/10 触发 ClearLocalCostmap（每次 hits=2）
- replay（稳态）：0/10

我对这个差异的解释是：
- recovery 很多时候是被“局部临界状态”触发的（局部震荡、短暂停滞、贴障碍导致局部规划失败）
- live 场景里存在实时系统扰动：调度/频率抖动、传感器噪声、Gazebo 步进波动等，可能把系统推到临界点 ⇒ 触发 recovery
- replay 把输入序列固定（尤其我取了稳态窗口），临界扰动更少 ⇒ recovery 更少

一句话总结：
- **replay 的价值不是让机器人更强，而是把输入固定下来，让我区分“算法问题”和“实时系统时序/噪声问题”。**

### 7.3 时间：replay 不一定更快
当前统计结果里：
- mean_time：两组几乎一样（31.25 vs 31.59）
- median_time：replay 反而更慢一些（28.52 vs 31.46）

我认为这里有一个合理的“怀疑点”（需要后续更理想条件验证）：
- replay 每次启动 Terminal A（bag play）后，TF / scan / LiDAR 等链路存在 **预热/缓冲阶段**  
  即使我已经用“先跑 5 次把缓冲时间算出来”这种方式做了处理，仍然无法保证每一次都能精确对齐“机器人真正开始可控导航”的起点时刻
- 这部分预热造成的时间损耗是 **不可控且波动的**，可能会把 replay 的 time_sec 往上顶一点
- 因此现在看到 replay 与 live 耗时几乎一致，并不能直接说明 replay 不可能更快；反而可以提出一个推断：
  - **如果 replay 的起跑时刻能被更稳定地对齐（例如严格等 TF/scan 稳定 + costmap 完成初始化 + 再发 goal），那么 replay 的耗时理论上可能更短一些**

> 我目前只把它当作“推断/怀疑点”，不把它写成结论。因为需要更严格的起跑对齐口径（比如增加一个可观测的 ready 条件）才能证明。

---

## 8) 结论
1) 在 Map1 目标 D 点上，live 与 replay（稳态）成功率均为 100%，可达性一致。  
2) recovery 触发率差异明显：live 为 30%，replay（稳态）为 0%，说明部分恢复触发与实时运行时的时序/噪声扰动有关。  
3) replay 当前并未表现出更短耗时（均值接近，中位数更慢），但存在“预热/起跑对齐不可控”带来的时间损耗怀疑点；在更理想的对齐条件下，replay 耗时可能更短，需要后续补充实验验证。


---

## 9) 备注：为什么 replay 要取稳态窗口
我在 replay 过程中观察到：回放启动阶段（冷启动）更容易出现开局 recovery（costmap/TF/scan 未热，BT 先走清理分支）。  
为了让对比聚焦在“导航行为”而不是“启动初始化抖动”，我本次对比采用 replay 的稳态 10 次作为样本（runs 6–15），并在 notes 里明确该规则。

---

## 10) 快速自检（避免把系统带偏）
### 10.1 回放自检（建议只做一次，确认链路没问题）
    ros2 topic info /clock -v | sed -n '1,25p'
    ros2 topic echo /tf --qos-reliability reliable --qos-durability volatile -n 1
    ros2 topic echo /scan --qos-reliability reliable --qos-durability volatile -n 1

要求：
- /clock Publisher count = 1
- /tf 与 /scan 能稳定刷到数据

### 10.2 BT log 自检（保证 recovery 能统计）
    wc -l /tmp/week5d6_map1_replay_bt.log
    wc -l /tmp/week5d6_map1_live_bt.log

要求：
- 行数随着每次 run 都在增长（否则脚本窗口为空，recovery 会被统计成 N/0）
