# ROS2 Nav2 Portfolio (TurtleBot3 / Humble)

[English](README_EN.md) | [中文](README.md)

A repository for reproducible learning and troubleshooting with **TurtleBot3 (TB3) + Nav2 (ROS2 Humble)** as the main thread, including standardized evaluation data, minimal controlled comparisons, cross-map generalization, a rosbag replay workflow, and a failure case library.

---

## Quick Start

### Demo (Nav2 system launch)
- Entry document: `docs/week1_day5_run_repro.md`

### Bag recording / replay (Week5 standardized procedure)
- Entry document: `docs/week5_day7_summary.md`

---

## Overview of Completed Work (Week1–Week8)

This repository advances week by week from “running successfully” to “reproducible, quantifiable, diagnosable, and well-documented”.

### Week1: Bring-up and baseline dataset
- Completed: Nav2 demo bring-up; fixed basic definitions for success/timeout.
- Outputs: reproduction documents and baseline evaluation data (fixed goal, reviewable).

### Week2: Pipeline explanation and evidence
- Completed: documented TF / costmap / planner-controller / BT recovery with parameters and observed evidence.
- Outputs: evidence documents for the corresponding modules (parameters, topic checks, and visualization checklist).

### Week3: Evaluation evidence-chain upgrade (Eval v0 → v2)
- Completed: upgraded from manual recording to a reproducible evaluation pipeline; recorded success/time and recovery behaviors.
- Outputs: evolution of Eval v0/v1/v2 and corresponding result datasets; minimal A/B comparison datasets.

### Week4: Minimal controlled experiments (same map and same start–goal, data-driven comparison)
- Completed: controlled comparison of six configurations on the same map with a fixed start and goal  
  (Baseline / A / B(theta=0.6) / B'(theta=0.4) / A+B(theta=0.6) / A+B'(theta=0.4)).
- Outputs: per-run datasets, summary table, and an interpretable conclusion document.

### Week5: Three-map generalization stress test and bag replay workflow
- Completed: evaluated generalization on Map1/2/3; established bag recording/replay; closed the recovery evidence chain (not relying on subjective observation).
- Outputs: per-map run logs and summary table; Live vs Replay comparison; recording/replay procedure document.

### Week6: Failure case library (five root-cause categories)
- Completed: consolidated representative failures into searchable cases using a unified structure (symptom–evidence–root cause–fix–verification).
- Outputs: five case documents and a failure index page.

### Week7: Portfolio-oriented repository structure (entry points and index)
- Completed: kept README as a single-screen entry; all details are linked through `docs/index` and weekly documents; no additional launch methods or new startup scripts were introduced; the existing reproduction protocol was preserved.
- Outputs: improved documentation index and Week7 packaging document.

### Week8: Ready Packet (single entry point with evidence binding)
- Completed: consolidated Week1–Week7 capabilities and evidence into a single entry page for quick review.
- Output: `docs/week8_ready_packet.md`.

---

## Final Deliverables

A reproducible Nav2 learning and troubleshooting portfolio repository, including:
- authoritative data entry points (baseline and summary tables) and raw per-run records;
- week-by-week process documentation (pipeline explanation, evaluation, controlled comparisons, generalization, replay);
- a failure case library (unified structure, searchable);
- a clear entry structure: README (single-screen entry) + `docs/index` (global index) + Week8 Packet (single entry page).

---

## Standardized Protocol (Repository-wide)

### Success / Timeout
- success: the reached condition is **distance to goal ≤ xy_goal_tolerance**
  - baseline: `xy_goal_tolerance = 0.25 m` (see `configs/baseline_burger.yaml -> goal_checker`)
- timeout: 120 s (fixed)

### Collision (Current Protocol)
- collision: not automatically quantified at present (retained as a simplified protocol). If upgraded, it will be documented in the corresponding weekly documents.

---

## Results (Data)

- Week1 Day6 baseline evaluation data: `results/w1d6_baseline.csv`
- Week3 evaluation and action logs: `results/week3_day*_*.csv`
- Week4 controlled experiment runs: `results/week4_*_runs.csv`
- Week4 summary table: `results/week4_summary_table.csv`
- Week5 Map1/2/3 runs:
  - `results/week5_map1_runs.csv`
  - `results/week5_map2_runs.csv`
  - `results/week5_map3_runs.csv`
- Week5 summary table: `results/week5_summary_table.csv`
- Week5D6 (Live vs Replay): `results/week5d6_*.csv`

---

## Documentation (Global Index)
- `docs/index_EN.md`

---

## Failure Library
- Entry: `docs/week6_failure_index.md`

---

## Directory Description (Current Structure)
- `assets/`: auxiliary resources (e.g., urdf)
- `bags/`: rosbag2 datasets
- `configs/`: parameter snapshots, replay QoS, RViz configurations
- `docs/`: weekly process documents and cases
- `maps/`: custom maps (pgm/yaml)
- `picture/`: key screenshots (TF/costmap/planner/controller/goals)
- `results/`: evaluation CSVs and summary tables
- `scripts/`: evaluation and summarization scripts
- `videos/`: demonstrations
