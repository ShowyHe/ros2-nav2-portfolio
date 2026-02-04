# Docs 总索引（Week1–Week8）

---

## Week1（跑通 + 定口径）
- docs/week1_day5_run_repro.md
- docs/week1_day6_baseline_eval.md
- docs/week1_day7_recap.md

---

## Week2（链路讲清楚：TF / costmap / planner-controller / BT recovery）
- docs/week2_day1_pipeline_onepager.md
- docs/week2_day2_tf_values.md
- docs/week2_day3_costmap_playbook.md
- docs/week2_day4_planner_controller_values.md
- docs/week2_day5_bt_recovery_values.md
- docs/week2_day6_baseline.md
- docs/week2_day7_summary.md

---

## Week3（评估脚本与 AB 对照：eval v0 → v2）
- docs/week3_day1_eval_v0.md
- docs/week3_day2_eval_v1.md
- docs/week3_day3_eval_v2.md
- docs/week3_day4_AB_test.md
- docs/week3_day5_summary.md
- docs/week3_day6_eval_v2.md
- docs/week3_day7_summary.md

---

## Week4（最小调参实验：baseline / A / B / A+B）
- docs/week4_day1_experiment_design.md
- docs/week4_day2_baseline.md
- docs/week4_day3_A_goal_tolerance.md
- docs/week4_day4_B_max_vel_theta.md
- docs/week4_day5_AplusB.md
- docs/week4_day6_summary.md
- docs/week4_day7_summary.md

---

## Week5（换场景泛化 + bag 回放）
- docs/week5_day1_maps_and_protocol.md
- docs/week5_day2_map1_20runs.md
- docs/week5_day3_map2-3_runs.md
- docs/week5_day4_map_generalization_report.md
- docs/week5_day5_bag_record.md
- docs/week5_day5_map1_bag_replay_setup.md
- docs/week5_day6_liveVSreplay.md
- docs/week5_day7_summary.md

---

## Week6（失败案例库）
- docs/week6_failure_index.md
- docs/week6_case01_map3_timeout_structural.md
- docs/week6_case02_controller_oscillation.md
- docs/week6_case03_costmap_clearlocal_storm.md
- docs/week6_case04_replay_timebase_qos.md
- docs/week6_case05_localization_initialization.md

---

## Week7（规约 + 入口 + 自检清单）
- docs/week7_summary_packaging.md

---

## Week8（总入口：证据绑定 Packet）
- docs/week8_ready_packet.md

## Results 入口（数据总入口）

### 入口
- Week1 基线：`results/w1d6_baseline.csv`
- Week4 汇总表：`results/week4_summary_table.csv`
- Week5 汇总表：`results/week5_summary_table.csv`

### 说明
- `runs.csv`：逐次运行的原始记录（证据数据）。
- `summary_table.csv`：阅读与对比的入口（结论入口）。
