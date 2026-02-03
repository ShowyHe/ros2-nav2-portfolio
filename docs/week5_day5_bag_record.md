# Week5 Day4 — Map1 录 bag 口径（一次成功样本即可）

> 今天第一个任务：**把“可回放 + 可复盘”的 bag 录出来**。  
> 录制路径：`bags/week5_map1_run01/`（里面有 `metadata.yaml` 和 `*.db3`）。

---

## 0. 录之前先把三件事想明白（不然白录）
1) **ROS_DOMAIN_ID 必须统一**：Gazebo / Nav2 / 录 bag / RViz 全部用同一个（我这里是 88）。
2) **/map 和 /tf_static 通常只发 1 条**：录制一定要覆盖到这两条消息出现的时间段。
3) **你录 bag 的目的分两种**：
   - **纯可视化回放**（看地图、激光、TF、里程计）：录 `/clock /tf /tf_static /scan /map /odom` 
   - **可复盘/可评估**（要成功/失败、耗时、recovery、action 状态）：必须额外录 action + BT 日志等。

---

## 1) Terminal A（Gazebo）
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

---

## 2) Terminal B（Nav2）
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

---

## 3) Terminal C（BT log 落盘：recovery 统计的“证据源”）
> 这一条是“以后复盘时你能锤死恢复发生过”的关键证据。

    : > /tmp/week5_map1_bt.log
    ros2 topic echo /behavior_tree_log >> /tmp/week5_map1_bt.log

---

## 4) Terminal D（开始录 bag：推荐“可复盘”口径）
> 你要的是“以后能回放 + 能解释结果 + 能复算指标”，就用这套。

### 4.1 录制命令（建议直接新 runXX，避免目录已存在的报错）
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    cd ~/ros2_nav2_portfolio
    mkdir -p bags

    ros2 bag record -o bags/week5_map1_run01 \
      --include-hidden-topics \
      /clock \
      /tf /tf_static \
      /odom \
      /scan \
      /map \
      /cmd_vel \
      /amcl_pose \
      /behavior_tree_log \
      /navigate_to_pose/_action/status

说明：
- `--include-hidden-topics`：不加这个，action 相关很多时候录不全。
- `/navigate_to_pose/_action/status`：最少得有它，不然后面没法稳定判定“结束态”。

### 4.2 如果你遇到 “output directory already exists”
两条路：
- 方案 A：换名字（最快）
    ros2 bag record -o bags/week5_map1_run02 --include-hidden-topics ...

- 方案 B：确认不要旧的就删（干净但有破坏性）
    rm -rf bags/week5_map1_run01
    然后重录 run01

---

## 5) 录制期间动作（只跑 1 次就行）
1) 打开 RViz
2) 2D Pose Estimate（如果需要）
3) 点一次 Nav2 Goal（去终点）
4) 等导航结束后再等 3–5 秒（让末尾状态、TF、日志刷完）
5) Terminal D 按 Ctrl+C 停止录制

---

## 6) 录完立刻验收（不验收=没录）
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    cd ~/ros2_nav2_portfolio
    ros2 bag info bags/week5_map1_run01 | egrep "Topic: /clock|Topic: /tf |Topic: /tf_static|Topic: /scan|Topic: /map|Topic: /odom|navigate_to_pose|behavior_tree_log"

至少要看到这些（最低合格线）：
- `/clock`（Count 很多）
- `/tf`（Count 很多）
- `/tf_static`（Count = 1 常见）
- `/scan`（Count 很多）
- `/map`（Count = 1 常见）
- `/navigate_to_pose/_action/status`（有就对了）
- `/behavior_tree_log`（有就对了）

---

## 7) 你现在这个 bag 的“现实情况”（别自欺欺人）
贴的 `ros2 bag info bags/week5_map1_run01` 已经说明：当前 run01 里只有
- `/scan` `/map` `/clock` `/tf` `/tf_static`

这类 bag 只能做“可视化回放”（地图/激光/TF），**不够做 action 结果复盘**。  

---

## 8) 保存路径口径（写进 README/MD 的统一说法）
- bag：`~/ros2_nav2_portfolio/bags/week5_map1_run01/`
- bt log：`/tmp/week5_map1_bt.log`

---

## 9) 常见坑
- ROS_DOMAIN_ID 不统一：录出来的 bag 缺 topic，回放时就各种“不存在/没数据”。
- 只录了 /map（1 条）但 RViz 没接住：回放时要么用 Transient Local，要么确保回放那一刻 RViz 已经订阅（你现在 Map 已经能显示，说明这块你已经踩过去了）。
- 目录已存在：不是系统坏了，是 rosbag2 默认不覆盖，换 run02 或删目录就完事。

---

## 10) 今日bag录制交付物
- 一条“成功录制”的命令（第 4 节原样贴）
- `ros2 bag info ...` 的 topic 列表截图/粘贴（证明录到了）
- /tmp/week5_map1_bt.log 存在（ls -lh 一下就行）
