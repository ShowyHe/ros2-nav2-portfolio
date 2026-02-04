# Week8 — 总入口（证据绑定 Packet）

> 本文件是 Week8 的唯一入口：把“问题 → 证据 → 结论”绑定起来。  
> 不新增启动脚本、不引入新的启动方式；复现口径以 Week1/Week5 既有文档为准。

---

## 0）周产出概览（Week1–Week7）

- **Week1：Nav2 跑通 + Baseline 数据**  
  证据入口：results/w1d6_baseline.csv  
  复现入口：docs/week1_day5_run_repro.md、docs/week1_day6_baseline_eval.md

- **Week2：从“能跑 demo”升级到“可解释、可复现、可量化”**  
  目标：每一层出问题都能给出证据（日志/参数/截图/表格）  
  证据入口：Week2 系列文档（TF / costmap / planner-controller / BT recovery / baseline）

- **Week3：评估证据链升级（Eval v0→v1→v2）+ 最小 A/B 对照**  
  目标：从手工表格升级为可复现、可量化、可解释 failure/recovery 的评估链路  
  证据入口：Week3 eval 文档与输出 CSV（以仓库现有文件为准）

- **Week4：同图同起终点的 6 组配置对照 + 汇总表**  
  配置组：Baseline / A / B(theta=0.6) / B'(theta=0.4) / A+B(theta=0.6) / A+B'(theta=0.4)  
  输出：可复现汇总表 + 可解释结论  
  入口：results/week4_summary_table.csv

- **Week5：三地图泛化压力 + bag 录制/回放打通 + recovery 证据闭环**  
  入口：results/week5_summary_table.csv  
  复现与口径入口：docs/week5_day7_summary.md

- **Week6：5 个 Case 失败库沉淀（可快速检索 + 可对照）**  
  入口：docs/week6_failure_index.md

- **Week7：作品集结构收敛（可浏览、可定位、可复盘）**  
  原则：README 只做“一屏入口”；细节由 docs 总索引与周文档承接  
  约束：不新增启动脚本、不引入新的启动方式；保持既有复现口径不变  
  入口：docs/index.md、README.md

---

## 1）权威入口（按“先看结论→再追证据”的顺序）
- **README（一屏入口）**：README.md  
- **Docs 总索引（Week1–Week7 导航）**：docs/index.md  

### Results 权威入口
- **Week1 基线**：results/w1d6_baseline.csv  
- **Week4 汇总表**：results/week4_summary_table.csv  
- **Week5 汇总表**：results/week5_summary_table.csv  

### 复现口径入口（不重复步骤）
- **Nav2 demo 启动（系统 launch 口径）**：docs/week1_day5_run_repro.md  
- **Week1 baseline 评估口径**：docs/week1_day6_baseline_eval.md  
- **Week5 bag 录制/回放口径 + recovery 证据闭环**：docs/week5_day7_summary.md  

### 失败库入口
- **Week6 Failure Library 索引**：docs/week6_failure_index.md

---

## 2）问题 → 证据 → 结论（证据绑定卡片）

> 约定：每张卡片只保留“可落到仓库文件”的证据，不写主观描述。

### 卡片 1：TF 是什么？如何验证 TF 主链与 use_sim_time 一致？
- **证据**：docs/week2_day2_tf_values.md、docs/week2_day1_pipeline_onepager.md  
- **结论**：TF/时间源不一致会导致定位、costmap、规划与可视化被同时污染；必须先完成 TF 主链与 use_sim_time 的证据检查，再讨论导航行为。

### 卡片 2：costmap 有哪些层？inflation 如何影响行为？如何用证据定位“保守/贴墙/封路”？
- **证据**：docs/week2_day3_costmap_playbook.md  
- **结论**：障碍层/膨胀层的表征变化可通过 RViz 层可视化与固定参数记录复现；当局部通道被“封死”或代价异常时，会表现为局部不可行与恢复触发。

### 卡片 3：planner vs controller 如何区分？怎么判断更像 planner 问题还是 controller/costmap 问题？
- **证据**：docs/week2_day4_planner_controller_values.md  
- **结论**：planner 负责 global path，controller 负责局部轨迹跟踪与避障；区分关键在于“全局路径是否稳定/局部轨迹是否可行且可持续跟踪”，并结合恢复类型与强度做归因。

### 卡片 4：Behavior Tree（BT）与 recovery 是什么？Clear/Spin/BackUp 各意味着什么？
- **证据**：docs/week2_day5_bt_recovery_values.md  
- **结论**：recovery 是系统在局部不可行/跟踪失败/表征异常时的自救动作；ClearLocalCostmap、Spin、BackUp 的组合与频率可以作为归因线索的一部分。

### 卡片 5：评估口径是什么？success/timeout 如何固定？结论入口在哪里？
- **证据**：docs/week1_day6_baseline_eval.md、results/w1d6_baseline.csv  
- **结论**：success 与 timeout 必须固定为全仓口径；逐次原始记录看 runs.csv（证据数据），结论对比统一看 summary_table.csv（权威入口）。

### 卡片 6：Week4 最小对照实验做了什么？结论入口在哪里？
- **证据**：docs/week4_day1_experiment_design.md（设计）、docs/week4_day2_baseline.md（baseline）、docs/week4_day3_A_goal_tolerance.md（A）、docs/week4_day4_B_max_vel_theta.md（B/B’）、docs/week4_day5_AplusB.md（A+B/A+B’）  
- **权威入口**：results/week4_summary_table.csv  
- **结论**：在同图同起终点约束下，对照 6 组配置输出可复现汇总表；结论以汇总表为准，避免主观描述。

### 卡片 7：Week5 做了什么泛化与回放工作？recovery 证据链如何闭环？
- **证据**：docs/week5_day7_summary.md  
- **权威入口**：results/week5_summary_table.csv  
- **结论**：三地图把“泛化压力”量化；bag 录制/回放链路打通后，recovery 证据不依赖主观观察，而是可在同口径数据与日志中闭环复核。

### 卡片 8：Week6 失败库包含哪些根因桶？入口在哪里？
- **证据**：docs/week6_failure_index.md  
- **结论**：将 5 个 Case 以“可快速检索 + 可对照”的方式沉淀，形成可复用的定位知识库。

---

## 3）简历用项目描述（两版，数字以仓库数据为准）

### 3.1 长版（3–4 行）
- 基于 TB3 + Nav2（ROS2 Humble）构建可复现学习与排障仓库：Week1 跑通并固定 baseline 数据与口径。  
- Week2 将 TF/costmap/planner-controller/BT recovery 链路沉淀为可解释、可复现的证据（日志/参数/截图/表格）。  
- Week3 完成评估证据链迭代（Eval v0→v1→v2）并做最小 A/B 对照；Week4 在同图同起终点下对照 6 组配置并输出可复现汇总表。  
- Week5 完成三地图泛化与 bag 录制/回放闭环；Week6 构建 5 个失败案例库；Week7 收敛为“一屏入口 + docs 总索引”的作品集结构，不引入新的启动方式。

### 3.2 短版（1 行）
- TB3+Nav2（Humble）可复现评估与排障：评估链路（v0→v2）+ 6 组对照（Week4）+ 三地图泛化与回放（Week5）+ 5 个失败库（Week6）+ 作品集结构收敛（Week7）。

---

## 4）复现入口（仅跳转，不重复步骤）
- Nav2 demo 启动：docs/week1_day5_run_repro.md  
- Week1 baseline 评估口径：docs/week1_day6_baseline_eval.md  
- Week5 bag 录制/回放与口径：docs/week5_day7_summary.md  

