# Week5 Day5（Map1）——bag 回放可视化口径 + 今日踩坑复盘（含 namespace 大坑）

> 今日任务就一件事：把 `week5_map1_run01` 的 **bag 回放在 RViz 里稳定可视化**（地图、TF、/odom、轨迹都能看），并把今天遇到的坑 + 之前卡很久的 `${namespace}` 坑，写成“照抄就能复现”的口径。

---

## 0）最终启动口径（固定 A/B/C/D）

### Terminal A（只负责回放 bag；不做检查）
**默认推荐：先别 loop（避免时间回拨导致 RViz 闪、Map 掉）**

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88

    # 清场
    pkill -f "ros2 bag play" || true
    pkill -f rosbag2_player || true
    sleep 1

    # 回放（不 loop，先稳定）
    ros2 bag play ~/ros2_nav2_portfolio/bags/week5_map1_run01 \
      --topics /clock /tf /tf_static /scan /map /odom

**确认稳定后如果一定要 loop（知道它可能带来“闪/Map消失”）再用：**

    ros2 bag play ~/ros2_nav2_portfolio/bags/week5_map1_run01 --loop \
      --topics /clock /tf /tf_static /scan /map /odom

---

### Terminal B（只做证据检查：/clock、/map、/tf、frame 名）
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88

    # /clock：发布者必须 = 1
    ros2 topic info -v /clock | sed -n '1,120p'

    # /map：发布者 >= 1（注意：bag 里 /map 可能只有 1 条）
    ros2 topic info -v /map | sed -n '1,120p'

    # /tf 必须能在 2 秒内刷出内容（证明 TF 在流）
    timeout 2 ros2 topic echo /tf | sed -n '1,40p'

    # 先看真实 frame 名（不要想当然写 map/odom/base_link）
    timeout 2 ros2 topic echo /tf | grep -E "frame_id:|child_frame_id:" | head -n 80

---

### Terminal C（可选，仅在需要时才开,robot不开也能看到坐标在跑）
#### C1）RobotModel 需要：发布 /robot_description（TRANSIENT_LOCAL）
> 只要 RobotModel 红、或者你怀疑 /robot_description 有 `${namespace}` 这种残留，就开这个。

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88

    python3 - <<'PY'
    import pathlib
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

    URDF = "/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf"

    class Pub(Node):
        def __init__(self):
            super().__init__("robot_description_pub")
            qos = QoSProfile(depth=1)
            qos.history = HistoryPolicy.KEEP_LAST
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(String, "/robot_description", qos)
            self.msg = String()
            self.msg.data = pathlib.Path(URDF).read_text()
            self.create_timer(0.5, self.tick)
            self.get_logger().info(f"Publishing /robot_description: {URDF}")

        def tick(self):
            self.pub.publish(self.msg)

    rclpy.init()
    rclpy.spin(Pub())
    rclpy.shutdown()
    PY

#### C2）如果回放里确实没有 map frame（纯可视化兜底）：补一个静态 map->odom
> 仅当你在 B 里确认 /tf 里根本没有 `map`，而你又想让 Fixed Frame = map 时才用。

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

---

### Terminal D（启动 RViz：用仓库 configs 里的文件）
> 路径固定：`~/ros2_nav2_portfolio/configs/week5_replay_map1.rviz`

    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    rviz2 -d ~/ros2_nav2_portfolio/configs/week5_replay_map1.rviz --ros-args -p use_sim_time:=True

---

## 1）今日踩坑复盘（问题是什么 + 怎么解决）

### 1.1 /clock Publisher count 变成 2（最容易把后面全带崩）
**现象**
- `ros2 topic info -v /clock` 看到 Publisher count: 2

**影响**
- 回放时间源不干净，TF/显示会莫名其妙“抖/闪/对不上时间”，后面排查全浪费。

**解决**
- A 必须先清场（pkill），再只启动一次回放。
- 回放方式只选一种：**要么播 bag 自带 /clock（推荐）**，要么用 `--clock` 注入，但不要混用。

**硬口径**
- B 里 `/clock` 发布者必须 = 1 才继续下一步。

---

### 1.2 /map “No map received” 或者“出来一下又没了”
这是今天最烦的坑，根因分两类：

#### A）你以为“map 消失”，其实是 RViz 订阅 QoS 没对上（错过那 1 条）
**证据**
- `ros2 bag info ...` 能看到：`Topic: /map ... Count: 1`（bag 里 /map 只有 1 条消息）
- `ros2 topic info -v /map` 看到发布端（rosbag2_player）是 `Durability: TRANSIENT_LOCAL`
- 但 RViz 的订阅端经常是 `Durability: VOLATILE`（对不上就会错过，错过就永远没）

**解决**
- RViz 的 Map Display：把订阅 QoS 改成能接到“缓存消息”的模式：
  - Displays → Map →（展开 Topic / QoS 相关项）
  - **Durability Policy = Transient Local**
- 同时把 **Use Timestamp 取消勾选**（回放里时间戳/TF 缓存很容易搞你）

**验证**
- B 里 `ros2 topic info -v /map`：Publisher count = 1 且 RViz 已经在订阅
- RViz 左侧 Map 状态稳定显示“已接收”，不是一闪就没。

#### B）你开了 `--loop`，时间回拨会导致 RViz “闪/丢显示”（看起来像 map 消失）
**现象**
- Map 出来几十秒/一分钟，过一会儿又没了；你 Ctrl+C 再播，Map 又闪一下。

**根因**
- `--loop` 会让 `/clock` 回到起点，RViz 的 TF buffer / 消息缓存会被时间回拨搞乱，表现就是“闪/丢”。

**解决**
- **排查阶段默认不 loop**：先用不带 `--loop` 的 A 命令把链路稳定下来。
- 真要 loop：接受它可能闪；最稳的做法是“loop + 需要时重启 RViz”，别指望它永远不抖。


---

### 1.3 “TF 闪 / RobotModel 红”：/robot_description 里残留 `${namespace}`


**现象**
- RViz 的 RobotModel：Status = Error（模型不显示 / 红）
- TF 列表里一堆机器人相关 frame（base_link、base_scan、wheel_*）显示 “No transform…”
- 关键证据：`/robot_description` 里能 grep 到 `${namespace}` 这类未展开占位符

**一句话根因**
- `/robot_description` 发布的不是最终 URDF，而是带变量的模板文本；RViz 不会替你展开 `${namespace}`，所以 RobotModel 解析失败，表现像“TF 断链”。

**证据怎么拿（硬口径）**
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    ros2 topic echo /robot_description --once --field data | grep -oE '\$\{[^}]+\}' | head

只要这里能看到 `${namespace}`，就别继续怀疑 TF，先把描述修干净。

**为什么我这里不能用 xacro 解决**
- 我本机的 `turtlebot3_description` 只有 `.urdf`，没有 `.urdf.xacro`，所以“照抄 xacro 路径 + namespace:=空”这条路在我这里是走不通的。

**解决（最终生效的做法）**
- 直接用本机存在的 `.urdf` 发布 `/robot_description`
- QoS 必须 TRANSIENT_LOCAL（RViz 后开也能收到）
- 命令就是上面 Terminal C1 那段脚本（照抄就行）

**验证（必须输出 0）**
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=88
    ros2 topic echo /robot_description --once --field data | grep -oE '\$\{[^}]+\}' | wc -l

输出 0 才算把 `${namespace}` 这个坑真正解决。

---

## 2）Map1 的 yaml 要不要“选”？
- **只做回放可视化**：不需要加载 yaml；bag 里本来就有 `/map`（只是可能只有 1 条，需要 RViz QoS 配合）。
- **想让 Map 永远稳定、别吃“/map 只有 1 条”的亏**：那就别回放 `/map`，改为运行 `map_server` 用 Map1 的 yaml 来发布 `/map`（这是另一条更工程化的路）。

---

## 3）今日完成标准（我自己验收用）
- B：`/clock` Publisher count = 1
- B：`timeout 2 ros2 topic echo /tf` 能刷出内容
- RViz：Map 稳定显示（不是一闪就没）
- RViz：能看到小车轨迹/路径，且和录 bag 的行为一致
- 可选：RobotModel 绿（如果我开了 C1 发布 /robot_description）

---

## 4）一句话总结
今天回放不稳，核心就两类：**时间源不干净（/clock 多发布者 or loop 回拨）** + **/map 只有 1 条但 RViz QoS 没对上（VOLATILE 错过就永远没）**。另外 RobotModel 红那种“像 TF 断链”的假象，十有八九是 `/robot_description` 里残留 `${namespace}`，先把描述修干净再谈 TF。
