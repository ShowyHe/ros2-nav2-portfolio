# Week7 — 作品集打包（入口收敛 + 总索引 + 归档规范）

## 目标
- 将仓库从“过程记录”收敛为“可浏览、可定位、可复盘”的作品集结构。
- README 只承担“一屏入口”，所有周内细节由 docs 总索引与周文档承接。
- 不新增启动脚本，不引入新的启动方式；保持既有复现口径不变。

---

## 1）README 口径（只做入口）
README 必须满足：
- 不重复终端 A/B/C/D 启动细节（细节只在周文档中维护）。
- 仅提供以下入口链接：
  1. Demo 入口（Week1）
  2. bag 录制/回放入口（Week5）
  3. Results 权威入口（Week1/Week4/Week5）
  4. Docs 总索引入口（docs/index.md）
  5. Failure Library 入口（Week6）

### 1.1 Demo 入口（保持唯一口径）
- 文档：`docs/week1_day5_run_repro.md`

### 1.2 bag 录制/回放入口（保持唯一口径）
- 文档：`docs/week5_day7_summary.md`

### 1.3 Results 权威入口（只列“入口”，不堆 runs 明细）
- Week1：`results/w1d6_baseline.csv`
- Week4：`results/week4_summary_table.csv`
- Week5：优先使用 `results/week5_summary_table.csv`
  - 若当前仓库未提供 week5_summary_table，则暂以 `results/week5_map1_runs.csv / week5_map2_runs.csv / week5_map3_runs.csv` 作为入口，并在后续补齐汇总表后替换。

### 1.4 Docs 总索引入口
- `docs/index.md`

### 1.5 Failure Library 入口
- `docs/week6_failure_index.md`

---

## 2）docs/index.md（全覆盖总索引）
docs/index.md 的职责：
- 作为 Week1–Week7 全部文档的“集中导航页”。
- 每个 Week 的文档按 Day 顺序列出。
- Week6 的 Case 库单独成组（failure_index + 5 个 case）。

验收标准：
- `ls docs/` 中所有 week1–week6 的 md 文件，都能在 docs/index.md 中找到入口（至少出现一次）。
- 索引使用可点击链接（推荐相对路径，如 `week3_day3_eval_v2.md`）。

---

## 3）归档规范（防止后续结构失控）
### 3.1 docs（过程文档）
- 文件命名：`week{N}_day{M}_<topic>.md`
- 周总结：`week{N}_day7_summary.md`
- Week6 案例：`week6_case0X_<bucket>.md`

### 3.2 results（数据）
- runs 明细：`results/week{N}_*_runs.csv` 或 `results/week{N}_map*_runs.csv`
- 汇总表：`results/week{N}_summary_table.csv`
- 规则：runs 是证据原始数据；summary_table 是阅读入口与结论入口。

### 3.3 picture（截图）
- 建议按主题分组：`tf/`、`costmap/`、`planner_controller/`、`bt_recovery/`、`goals/`
- 文件名建议包含 week/day 与对象：`week5_day6_local_costmap.png`

### 3.4 bags（rosbag2）
- 单次样本：`bags/week{N}_map{K}_run{XX}`
- 批量录制：`bags/week{N}_map{K}_batch{XX}`
- bag 的用途与对应 runs 记录只在 Week5 文档中维护，README 不重复。

### 3.5 configs（配置快照）
README 中仅强调“跨周通用的基线快照”，避免引入过多周内临时文件：
- `configs/baseline_burger.yaml`
- `configs/map.yaml`
- `configs/tb3_navigation2.rviz`（可选）
- `configs/qos_bag_play.yaml`（如回放需要）

周内专用（例如 Week5 的临时参数/回放 RViz）如需保留：
- 保留文件本身可以，但不在 README 的“配置快照”中强调；
- 只在对应周文档（Week5）内部引用。

---

## 4）最小自检清单（不跑仿真）
在仓库根目录执行以下检查，全部通过即认为 Week7 打包完成：

- `test -f README.md`
- `test -f docs/index.md`
- `test -f docs/week1_day5_run_repro.md`
- `test -f docs/week5_day7_summary.md`
- `test -f docs/week6_failure_index.md`
- `test -f docs/week6_case01_map3_timeout_structural.md`
- `test -f docs/week6_case02_controller_oscillation.md`
- `test -f docs/week6_case03_costmap_clearlocal_storm.md`
- `test -f docs/week6_case04_replay_timebase_qos.md`
- `test -f docs/week6_case05_localization_initialization.md`
- `test -f results/week4_summary_table.csv`
- `test -f results/w1d6_baseline.csv`
- Week5 汇总入口（二选一）：
  - `test -f results/week5_summary_table.csv`
  - 或 `test -f results/week5_map1_runs.csv && test -f results/week5_map2_runs.csv && test -f results/week5_map3_runs.csv`

---

## 5）结论
Week7 的工作完成标志：
- README 一屏入口清晰、无重复启动流程、无多余脚本入口。
- docs/index.md 对 Week1–Week6 文档全覆盖，导航不遗漏。
- Results 有权威入口（至少 Week1/Week4/Week5 具备可直达入口）。
- 失败案例库入口明确，Case 文档可直接定位与复盘。
