# Week2 Day2 — TF 关键证据（TB3 + Nav2）

> 目标：用最少命令证明 **TF 主链活着**，并确认 **use_sim_time 全链一致**  
> 证据图：`picture/tf/week2_day2_frames.png`（或 pdf）

---

## TF 排查剧本
> 一句话：**TF 是导航的“空间共识地基”**。地基断了，costmap / planner / controller 全都会表现为“不动/抽风”。  
> 目标：用 3 步把问题从“玄学”压到“具体节点 / 具体链路 / 具体参数”。
1.先判 TF 有没有在刷、刷得稳不稳
看 /tf 频率和 RViz 报不报 transform error —— TF 不刷，后面全白搭。
2.再判 TF 主链是不是通的（哪一段断了）
按顺序验：map→odom（定位/AMCL）→ odom→base_footprint（底盘里程计）→ base_link→base_scan（传感器静态TF）。
哪段不通，就把问题收敛到对应模块。
3.最后判时间（use_sim_time）是不是统一
只要出现 extrapolation / “过去未来外插”，优先怀疑 sim_time 不一致；把 AMCL、controller_server、bt_navigator 这些关键节点都确认成 True。

## A. TF 主链是否通（导航“地基”）

### A1) map → odom（定位链是否通：AMCL 负责）
命令：
```bash
ros2 run tf2_ros tf2_echo map odom
```
 
[INFO] [1768877178.822765182] [tf2_echo]: Waiting for transform map ->  odom: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
At time 7137.554000000
- Translation: [-0.000, 0.413, 0.003]
- Rotation: in Quaternion (xyzw) [-0.000, -0.003, -0.014, 1.000]
- Rotation: in RPY (radian) [-0.000, -0.006, -0.028]
- Rotation: in RPY (degree) [-0.006, -0.329, -1.623]
- Matrix:
  1.000  0.028 -0.006 -0.000
 -0.028  1.000  0.000  0.413
  0.006 -0.000  1.000  0.003
  0.000  0.000  0.000  1.000

正常情况：持续输出 At time ...，Translation/Rotation 会缓慢变化（AMCL 在纠偏）
错误情况：
一直报 Invalid frame ID "map" 或 Could not transform，且一直没有 At time ...
→ AMCL 没起来 / 没发布 TF / 初始位姿没设 / use_sim_time 不一致

报 Lookup would require extrapolation into the past/future
→ 时间问题（Gazebo 时钟、use_sim_time、或 tf buffer 时间戳不一致）

### A2) odom → base_footprint（底盘里程计链是否通）
命令：
```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

[INFO] [1768877773.500286163] [tf2_echo]: Waiting for transform odom ->  base_footprint: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
At time 7730.715000000
- Translation: [-1.999, -0.500, 0.009]
- Rotation: in Quaternion (xyzw) [0.000, 0.003, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, 0.006, 0.000]
- Rotation: in RPY (degree) [0.006, 0.328, 0.011]
- Matrix:
  1.000 -0.000  0.006 -1.999
  0.000  1.000 -0.000 -0.500
 -0.006  0.000  1.000  0.009
  0.000  0.000  0.000  1.000

正常情况：机器人动时，这个 Translation 会变化；机器人不动时，也应该能持续输出（数值稳定）
错误情况：
一直没 At time ...
→ Gazebo/仿真底盘没起来、diff_drive/里程计节点不工作、TF 没发布

数值跳得很怪（突然爆大、NaN）
→ 里程计异常/仿真不稳/时间不一致

## B. 传感器 TF 是否通（costmap 障碍层的前提,机器人本体到雷达坐标的坐标变换）
B1) base_link → base_scan（雷达坐标挂得对不对）
命令：
```bash
ros2 run tf2_ros tf2_echo base_link base_scan
```

[INFO] [1768877684.422200937] [tf2_echo]: Waiting for transform base_link ->  base_scan: Invalid frame ID "base_link" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [-0.032, 0.000, 0.172]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000 -0.032
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.172
  0.000  0.000  0.000  1.000

正常情况：大概率 At time 0.0（静态）或固定时间戳；数值小且固定
错误情况：
报 frame 不存在，且一直没有 At time 0.0
→ robot_state_publisher 没起 / URDF 没加载 / frame 名写错（有的叫 scan 不是 base_scan）

数值特别离谱（比如 x=3m、z=-1m）
→ 传感器安装 TF 配错，会导致 costmap “障碍乱飞/偏移”

## C. TF 发布是否稳定（频率证据）
C1) /tf 频率（不追求完美，只要别离谱）
命令：
```bash
ros2 topic hz /tf
```

average rate: 53.824
        min: 0.000s max: 0.035s std dev: 0.01674s window: 55
average rate: 53.852
        min: 0.000s max: 0.035s std dev: 0.01674s window: 110

正常情况：没有绝对标准，但别低到个位数；也别出现“长时间 0Hz”。
错误情况：
average rate 很低（比如 1~5Hz）或经常断流
→ 仿真卡、CPU 满、某关键 TF 发布源挂了、时间不一致

频繁报 extrapolation
→ 大概率还是 use_sim_time/时钟 或 TF buffer 里的时间戳问题

## D. use_sim_time 是否一致（时钟不一致=全链出错）
D1) AMCL use_sim_time
命令：
```bash
ros2 param get /amcl use_sim_time
```

Boolean value is: True

错误情况：
报False，可能出现 TF 时间错乱，进而导致规划/控制间歇性失败
→ launch 参数没传进去 / 你手动起的节点没带 use_sim_time:=True
→ 修复：统一从同一个 launch 起，或给该节点显式设置 use_sim_time

D2) controller_server use_sim_time
命令：
```bash
ros2 param get /controller_server use_sim_time
```

Boolean value is: True

D3) bt_navigator use_sim_time
命令：
```bash
ros2 param get /bt_navigator use_sim_time
```

Boolean value is: True

