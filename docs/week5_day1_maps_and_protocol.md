# Week5 Day1 — 三地图战场与评估口径（Maps + Protocol）

## 0. 本日文件清单（仅限 Day1 产出）
- `docs/week5_day1_maps_and_protocol.md`（本文）
- `maps/week5_map2_mid_house.yaml`
- `maps/week5_map2_mid_house.pgm`
- `maps/week5_map3_narrow_house.yaml`
- `maps/week5_map3_narrow_house.pgm`

---

## 1. 三张地图定义（Map1 / Map2 / Map3）

### Map1（最简单，官方基线对照）
- Gazebo World：`turtlebot3_world`
- 地图来源：官方示例（由 `map_server` 发布 `/map`，无需本仓库提供 yaml/pgm）
- 目的：作为“最稳基线”，用于验证评估口径、流程与脚本在标准场景下稳定工作。

### Map2（中等，长廊加直角障碍）
- Gazebo World：`turtlebot3_house`
- 地图文件：
  - `maps/week5_map2_mid_house.yaml`
  - `maps/week5_map2_mid_house.pgm`
- 目的：体现中等复杂度（走廊 + 90°拐弯 + 中等门洞 + 少量障碍），用于观察 controller 与 recovery 在“非极限场景”下的行为。

### Map3（最难，自建窄门压力图）
- Gazebo World：`turtlebot3_house`
- 地图文件：
  - `maps/week5_map3_narrow_house.yaml`
  - `maps/week5_map3_narrow_house.pgm`
- 目的：窄门 + 直角拐弯密集，作为压力测试场景，用于暴露局部规划与恢复策略的边界问题。

---

## 2. 起点 / 终点
> 原则：每张图固定一组起点/终点，Week5 全周不再更改。  
> Day1 仅预留字段，待 Day2/Day3 实验跑通后再回填坐标与截图证据。

- Map1 起点：`(x=?, y=?, yaw=?)`；终点：`(x=?, y=?, yaw=?)`
- Map2 起点：`(x=?, y=?, yaw=?)`；终点：`(x=?, y=?, yaw=?)`
- Map3 起点：`(x=?, y=?, yaw=?)`；终点：`(x=?, y=?, yaw=?)`

---

## 3. 单次导航判据（固定口径）

### 3.1 Success（S）
- action 返回 `SUCCEEDED`（或等价成功状态）
- 记录：`result=S`

### 3.2 Timeout（T）
- 单次导航运行时间超过 `120s` 未成功，判定超时
- 记录：`result=T`

### 3.3 Collision（C）
- 出现明确碰撞导致无法继续有效前进（例如顶墙卡死/持续抖动无法脱离），判定碰撞
- 记录：`result=C`

---

## 4. Runs 数量与输出命名

### 4.1 Runs
- 每张地图：`runs = 20`

### 4.2 CSV 输出命名
- `results/week5_map1_runs.csv`
- `results/week5_map2_runs.csv`
- `results/week5_map3_runs.csv`

---

## 5. 一键启动口径（按 Map1→Map2→Map3 顺序）

### 5.1 Map1 — turtlebot3_world（官方基线）
Terminal A（Gazebo / Map1）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal B（Nav2 / Map1）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

---

### 5.2 Map2 — mid_house（自建中等图）
Terminal A（Gazebo / Map2）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

Terminal B（Nav2 / Map2）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/hexiaoyi/ros2_nav2_portfolio/maps/week5_map2_mid_house.yaml

---

### 5.3 Map3 — narrow_house（自建窄门压力图）
Terminal A（Gazebo / Map3）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

Terminal B（Nav2 / Map3）：
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/hexiaoyi/ros2_nav2_portfolio/maps/week5_map3_narrow_house.yaml

---

## 6. 地图发布体检四连（通用，10 秒定位“RViz 地图为空”）
> 判定标准：`map_server` 存在 + lifecycle 为 `active` + `yaml_filename` 路径正确 + `/map` 的 `Publisher count >= 1`。

    # 1) map_server 在不在
    ros2 node list | grep -E "^/map_server$|map_server"

    # 2) 生命周期是否 active（必须 active 才会发布 /map）
    ros2 lifecycle get /map_server

    # 3) 它加载的 yaml 路径（Map2/Map3 必须指向本仓库 maps/ 下对应文件）
    ros2 param get /map_server yaml_filename

    # 4) /map 是否有 publisher（Publisher count 必须 >= 1）
    ros2 topic info /map -v | sed -n '1,25p'
