# Week5 Day3 — Map2 / Map3 泛化评估（口径固定：timeout=120s + recovery 统计）

## 1. 今日目标
- 用**同一套 Nav2 配置**在不同地图上跑评估，比较泛化差异
- 固定 timeout=120s
- 固定脚本：`scripts/eval_v2_action_recovery.py`
- 产出：Map2 / Map3 两份 `runs.csv` + 本日结论（成功/失败/超时分布 + hits 分布）

---

## 2. 输入与输出（文件级口径）
### 输入
- 地图：Map2、Map3
- 点位：
  - Map3：start=(1, 3)，goal=(6.15, -1.15)
- 日志：
  - Map2：`/tmp/week5_map2_bt.log`（如果你用的是这个名字）
  - Map3：`/tmp/week5_map3_bt.log`

### 输出（原始数据不在 md 里重复）
- Map2 原始 runs：
  - `results/week5_map2_runs.csv`（或你实际命名的 Map2 文件）
- Map3 原始 runs：
  - `results/week5_map3_runs.csv`

---

## 3. 评估口径（写死）
### 3.1 `result` 字段
- `S`：在 timeout 内捕捉到 action 终态为成功（SUCCEEDED）
- `F`：在 timeout 内捕捉到 action 终态为失败/取消（FAILED/ABORTED/CANCELED 等）
- `T`：timeout 内未捕捉到 action 终态（统一记为 TIMEOUT）

### 3.2 `recovery_hits` 字段
- `recovery_hits` 是脚本从 BT log 里按 `configs/recovery_keywords.txt` 统计出的**关键词命中次数**
- 这个 hits 口径 **≠ RViz 面板里显示的 recoveries 次数**（两者统计对象不同）

---

## 4. 结论

### Map2（总体趋势）
- **成功占多数**（S 多）
- 但 **recovery hits 偏多**（过程不干净，依赖 ClearCostmap / Spin / BackUp 等恢复）

### Map3（总体趋势）
- **TIMEOUT（T）占多数**，且 hits 普遍更高（例如常见 18/20）
- 但 Map3 的 `T` 需要额外说明：  
  - 有些 `T` 是 **视觉上已到达/基本到达**，只是 120s 内脚本没抓到终态  
  - 有些 `T` 是 **确实未到终点**  
- 所以 Map3 的 TIMEOUT 不能简单等价“失败”，需要在 `notes` 里补充 `VISUAL=reached / not_reached`

---

## 5. 问题整理
### 为什么 RViz 里看到 recoveries=9，但脚本统计 hits=18？
- RViz 的 recoveries 更像是“恢复阶段/触发次数”的 UI 汇总
- 脚本的 hits 是**日志关键词命中次数**：一次恢复过程中同一个关键词可能出现多次（比如 ClearLocalCostmap 在一个恢复周期内打印多次）
- 所以：两者不矛盾，口径不同

---

## 6. Day3 交付物清单
- `results/week5_map2_runs.csv`
- `results/week5_map3_runs.csv`
- `docs/week5_day3_generalization_map2_map3.md`（本文）

--- 

## 7. Week5 Day4该做什么
- 把 Map1 / Map2 / Map3 三张地图的结果做成一个汇总表（成功率 / TIMEOUT 比例 / hits 均值与分布）
- 输出“泛化结论”：哪张地图最稳、哪张最吃力、差异体现在哪里（用数据说话）

