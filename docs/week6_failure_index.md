# Week6 Failure Library — Index

> 目的：把 Week6 的 5 个 Case 以“可快速检索 + 可对照”的方式沉淀下来。  
> 口径：timeout=120s；success 判据沿用仓库 Week1 固定口径。

---

## Case 列表（链接）
1. [Case01 — Map3 系统性超时（结构性不可行）](docs/week6_case01_map3_timeout_structural.md)
2. [Case02 — Map2 Controller Oscillation（Spin/BackUp 风暴）](docs/week6_case02_controller_oscillation.md)
3. [Case03 — Map2 Costmap / Perception（ClearLocalCostmap 高频）](docs/week6_case03_costmap_clearlocal_storm.md)
4. [Case04 — Replay / Timebase / QoS（回放链路类故障）](docs/week6_case04_replay_timebase_qos.md)
5. [Case05 — Localization / Initialization（AMCL 未收敛数据污染）](docs/week6_case05_localization_initialization.md)

---

## 汇总表（现象 / 根因桶 / 修复动作 / 指标变化）
| Case | 现象（可观察） | 根因桶（归类） | 修复动作清单（最小闭环） | 指标变化（来自 Week5 数据/或规约） |
|---|---|---|---|---|
| Case01 Map3 系统性超时 | 19/20 固定 120s 超时；多数标注 VISUAL=reached 仍超时；恢复常态化、强度高 | 结构性不可行：局部可行性/全局-局部一致性/目标邻域收敛失败 | 固定口径→检查 global path 连续性→检查 local 轨迹可行性→检查目标邻域收敛→以超时率下降为唯一验收 | success=5%（1/20）；timeout=95%（19/20）；超时样本 recovery=100%（19/19）；recovery_hits median=18 |
| Case02 Controller Oscillation | 超时/失败样本 Spin/BackUp 明显；存在高 hits 循环（run9 hits=31） | 控制层/局部可行性不稳定：轨迹跟踪失败导致恢复链循环 | 固定口径→观测 cmd_vel 行为→核查局部轨迹生成/失效→核查 local costmap 是否封路→以 Spin/BackUp 循环消失为验收 | Map2：success=75%（15/20）；timeout=20%（4/20）；fail=5%（1/20）；Spin 出现 6/20；BackUp 出现 6/20；峰值 hits=31 |
| Case03 Costmap / Perception | ClearLocalCostmap 在 Map2 成功样本中也高频（多次 hits=2 重复） | 代价地图表征不稳定/逼近恢复边缘：局部 costmap 常将路径逼到不可行边缘 | 固定口径→核查 obstacle/inflation 是否封路/抖动→核查清理触发点是否集中→检查与控制层耦合→以 ClearLocal 频次下降与超时减少为验收 | ClearLocal：Map2 为 80%（16/20），Map1 为 30%（6/20）；Map1 触发均 hits=2；Map2 成功样本大量 hits=2 重复 |
| Case04 Replay / Timebase / QoS | 回放中 TF/RViz/Map 显示不稳、闪烁、No map received 等“假异常” | 观测链路问题：时间源冲突、时间回拨、QoS 不匹配 | /clock 发布者=1→use_sim_time 全链一致→/map QoS 匹配（Transient Local）→TF 连续性检查→健康条件达标后再观察上层 | 规约型 Case：健康条件不满足时，现象与数据判为不可用观测；先修复观测链路再记录 |
| Case05 Localization / Initialization | 初始阶段位姿/TF 不稳导致 costmap 对齐异常；runs 统计被“定位自救”混入而波动 | 数据污染：AMCL 未收敛导致 map→odom 不稳定，导航行为与统计不可解释 | 收敛判定三连（amcl_pose 稳 / map→odom 稳 / 对齐一致）→通过后再记录→中途漂移直接判无效样本→重跑 | 规约型 Case：未通过收敛判定的 run 直接判无效；示例锚点 Map3 run1（BT 区间可检查） |

---

## 定位方法四步法（简短）
1. 现象：先用统一口径描述（S/T/F、120s、是否可视接近、恢复类型/强度）。  
2. 证据：只取最小证据集（runs 表字段 + 必要的话题/TF/QoS 检查）。  
3. 定位：把问题归类到根因桶（结构性可行性 / 控制层 / costmap 表征 / timebase&QoS / 定位收敛）。  
4. 验证：给出最小闭环（修复动作 → 复跑对照 → 指标变化作为验收）。

