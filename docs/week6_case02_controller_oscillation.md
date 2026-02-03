# Week6 Case02 — Map2 Controller Oscillation（Spin/BackUp 风暴：局部控制层卡住）

## 0）Case 定位
1. Case 名称：Controller Oscillation（局部控制震荡/跟踪失败 → Spin/BackUp 循环）
2. 目标：用 Map2 runs 数据证明失败主要表现为“控制层恢复链”，并给出面试可复述的因果链与后续修复清单
3. 数据范围：Week5 Map2 共 20 runs（含 result/time_sec/recovery_types/recovery_hits）

---

## 1）现象
1. Map2 总体表现为“可跑但不稳”：20 runs 中成功 15 次，超时 4 次，失败 1 次。
2. 失败样本呈现明显“控制层恢复链”：Spin/BackUp 高频出现，常与 ClearLocal/ClearGlobal 叠加。
3. 存在教科书级“震荡卡死”样本：恢复 hits 极高，最终 120s 仍无法结束。

---

## 2）证据
### 2.1 结果与时间（硬事实）
1. 结果分布：
   - Success：15/20（75%）
   - Timeout：4/20（20%）—— run4 / run8 / run11 / run12
   - Fail：1/20（5%）—— run9
2. 耗时（成功样本的典型量级）：
   - 多数成功在 49–66s 区间（如 run3=49.361、run2=56.881、run1=61.261、run6=65.891）
   - 存在偏慢但仍成功的样本（run20=99.111、run13=82.371）

### 2.2 恢复触发率与类型结构
1. recovery=Y：16/20（80%）—— Map2 的“自救”几乎是常态。
2. recovery_types 统计（按 run 是否出现计数）：
   - ClearLocalCostmap：16 次
   - Spin：6 次（run1/run4/run8/run9/run12/run20）
   - BackUp：6 次（run4/run8/run9/run11/run12/run13）
   - ClearGlobalCostmap：5 次（run4/run8/run9/run12/run20）

### 2.3 关键锚点（用于落锤）
1. 教科书级失败样本（强锚点）：
   - run9：F，120.000s，types=BackUp;ClearGlobalCostmap;ClearLocalCostmap;Spin，hits=31
2. 典型超时组（强锚点）：
   - run4：T，120.000s，types=BackUp;ClearGlobal;ClearLocal;Spin，hits=9
   - run8：T，120.000s，types=BackUp;ClearGlobal;ClearLocal;Spin，hits=11
   - run12：T，114.499s，types=ClearLocal;BackUp;ClearGlobal;Spin，hits=16
   - run11：T，114.294s，types=ClearLocal;BackUp，hits=9
3. 成功但出现控制恢复链（弱锚点，用于说明“症状在成功样本中也出现”）：
   - run1：S，61.261s，types=ClearLocalCostmap;Spin，hits=5
   - run13：S，82.371s，types=ClearLocalCostmap;BackUp，hits=7
   - run20：S，99.111s，types=ClearLocalCostmap;ClearGlobalCostmap;Spin，hits=13

---

## 3）根因判断
1. 结论级判断：Map2 的主要失败形态是控制层/局部可行性不稳定引发的恢复链循环（Spin/BackUp 作为主症状）。
2. 依据：
   1) Spin/BackUp 在超时与失败样本中高频出现，并与 hits 增大强相关（run9 hits=31；run12 hits=16；run8 hits=11）。
   2) run11 仅 ClearLocal+BackUp（无 Spin）也能进入 114s 超时，说明“局部不可行 → 后退自救”本身就可能导致无法收敛。
   3) 同一地图下存在大量成功样本，但失败样本具有清晰的“控制恢复链”特征，符合“特定几何结构/局部轨迹跟踪不稳定”触发条件化失败。
3. 面试可复述的因果链：
   - 观察到 `Spin/BackUp` 频繁出现 + `recovery_hits` 明显增大 + 结果进入 `T/F`
   - 归纳为：局部轨迹跟踪/局部避障不稳定 → 控制层触发 Spin/BackUp 自救 → 进入循环 → 超时或失败

---

## 4）修复动作清单与验证口径
### 4.1 验证口径（先固定）
1. timeout 固定：120s（与仓库统一口径一致）。
2. 成功判据：沿用仓库统一口径（Week1 固定的 success_radius）。
3. 对照原则：同一 Map2、同一起终点、同一套启动流程；只允许最小改动并做对照 runs。

### 4.2 修复动作清单（按优先级）
1. 控制层观测对齐（先确认“卡住时控制输出是什么”）
   - 验证点：卡住阶段 `/cmd_vel` 是否出现角速度反复拉满/来回跳；线速度是否被频繁压为 0 或反复后退。
2. 局部轨迹可行性检查（判断“是不是局部轨迹在几何结构下不可行”）
   - 验证点：局部轨迹是否反复生成又失效；机器人是否在同一位置附近反复触发 Spin/BackUp。
3. costmap 与局部规划耦合检查（判断“是不是局部 costmap 把通道封死导致控制无解”）
   - 验证点：Spin/BackUp 发生点附近，local costmap 是否出现膨胀封路/障碍层不稳定。
4. 恢复链节流（作为辅助，不当作根因修复）
   - 验证点：降低恢复次数不是目标；目标是减少进入 Spin/BackUp 循环并提升完成率/降低超时。

### 4.3 验证方式（最小对照）
1. 以 run9 所在路径结构为重点，做 baseline 组与修复组各 10 runs：
   - 指标：success_rate、median_time、recovery_hits（中位数/均值）、Spin/BackUp 出现频次（按 run 计数）。
2. 重点验收条件：
   - run9 类失败不再出现（hits 显著下降，且不再进入 120s F/T）。
   - Map2 的超时率显著下降（4/20 → 更低）。

---

## 5）复现入口
1. 数据入口：Week5 Map2 runs 记录（本文件引用的 20 runs）。
2. 关键锚点 runs：
   1) 主锚点：run9（F,120,hits=31，Spin/BackUp/清图全套）
   2) 超时锚点：run4/run8/run11/run12（T≈114–120，恢复链清晰）
   3) 成功但有控制恢复链：run1/run13/run20（用于说明症状并非只在失败样本出现）

---

## 6）结论
1. Map2 的失败更符合“控制层/局部可行性不稳定”而非纯地图难度：Spin/BackUp 在关键失败样本中高频出现且与 hits 增大一致。
2. run9（hits=31）是典型的控制震荡卡死样本：恢复链循环持续存在，最终 120s 仍无法结束，可作为面试回答“原地旋/后退循环怎么定位”的核心证据锚点。
3. 后续修复应优先围绕：控制输出行为（cmd_vel）、局部轨迹可行性、local costmap 是否封路三条主线推进；修复的有效判据是 Map2 超时/失败样本显著减少且 Spin/BackUp 循环不再出现。
