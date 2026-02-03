# Week6 Case01 — Map3 系统性超时（结构性不可行）

## 0）Case 定位
- Case 名称：Map3 系统性超时（结构性不可行）
- 目标：用 runs 数据证明这是“系统性不可行”，并给出面试可复述的因果链与后续修复清单
- 数据范围：Week5 Map3 共 20 runs（含 result/time_sec/notes/recovery_types/recovery_hits）

---

## 1）现象
1. 结果分布异常：20 runs 中 19 次固定 120s 超时，仅 1 次成功（run20：67.186s）。
2. 超时并非“跑不到”：多数超时 run 标注 `TIMEOUT:VISUAL=reached`，但依然卡死到 120s。
3. 超时伴随高强度恢复：19 次超时全部触发 recovery（recovery=Y），且 recovery_hits 长期处于高位。

---

## 2）证据
### 2.1 结果与时间（硬事实）
1. `timeout_rate = 19/20 = 95%`：run1–19 均为 `T, 120.000s`。
2. `success_rate = 1/20 = 5%`：run20 为 `S, 67.186s`（recovery=N）。

### 2.2 VISUAL reached / not_reached（来自 notes）
1. 在 19 次超时中：`VISUAL=reached` 共 16 次。
2. `VISUAL=not_reached` 共 3 次：run5、run13、run16。

### 2.3 恢复触发与强度（recovery_types / hits）
1. 在 19 次超时中：`recovery=Y` 为 19 次（100%）。
2. recovery_hits（仅统计 19 次超时）：
   - median = 18
   - mean = 16.37
   - min = 7：run10 / run15 / run19
   - max = 20：run3 / run16
3. recovery_types 结构：
   - 主流类型组合：`ClearLocalCostmap; BackUp; ClearGlobalCostmap; Spin`
   - 关键反例：恢复类型更少也超时
     - run10：`ClearLocalCostmap;BackUp`，hits=7，T=120
     - run15：`ClearLocalCostmap;BackUp`，hits=7，T=120
     - run19：`ClearLocalCostmap;BackUp`，hits=7，T=120

---

## 3）根因判断
1. 结论级判断：Map3 超时是系统性不可行，不是偶发波动。
   - 依据：19/20 固定 120s 超时，且超时样本 100% 触发恢复链。
2. 结构性问题指向：局部可行性或全局-局部一致性存在冲突，导致无法进入可终止轨迹。
   - 依据A：大量 `VISUAL=reached` 仍不结束，说明并非单纯“走不到”，更像“目标邻域收敛失败/终止条件无法满足/局部规划反复否决”。
   - 依据B：恢复更少（hits=7）仍超时，排除“恢复过多耗尽 120s”作为主要原因。
3. 面试可复述的因果链：
   - `VISUAL reached/not_reached` + `recovery_types/hits` + `固定 120s`
   - 归纳：恢复链高频叠加仍无法终止 → 结构性不可行。

---

## 4）修复动作清单与验证口径
### 4.1 验证口径（先固定，避免数据污染）
1. timeout 固定：120s。
2. 成功判据：沿用仓库统一口径（Week1 口径，success_radius 固定值不在本 Case 改动）。
3. 数据有效性口径：定位收敛后再开始计时与记录（避免 AMCL 初始化污染 runs）。

### 4.2 修复动作清单（按优先级）
1. 全局-局部一致性检查（先判“是不是 planner 问题”）
   - 验证点：是否持续存在可用 global path；global path 是否频繁重规划/跳变。
2. 局部可行性检查（再判“是不是 controller/costmap 问题”）
   - 验证点：局部轨迹是否反复抖动、断裂、贴障导致 BackUp/Spin 循环。
3. 目标邻域收敛检查（专门针对 VISUAL=reached 仍超时）
   - 验证点：到目标附近后是否出现“来回绕圈/旋转/后退”并持续触发恢复，无法满足终止条件。
4. 恢复链控制（只作为辅助，不当作根因）
   - 验证点：减少恢复次数不应被视为“修复成功”，必须以超时率下降/成功率提升为准。

### 4.3 验证方式（最小对照）
1. 选取 Map3 固定起终点，做 baseline 组与修复组各 10 runs：
   - 指标：success_rate、median_time、recovery_hits（中位数/均值）。
2. 必要对照：对比 Map3 与 Map1（Map1 为稳定对照场景）：
   - 判断修复是否“只对 Map1 有效”，还是能实质降低 Map3 结构性超时。

---

## 5）复现入口
- 数据入口：Week5 Map3 runs 记录（本文件引用的 20 runs 即为复现基准）。
- 关键锚点 runs：
  1. 结构性超时主样本：run1–19（T=120）
  2. `VISUAL=not_reached` 对照：run5 / run13 / run16
  3. “恢复少仍超时”反例：run10 / run15 / run19（hits=7）
  4. 唯一成功对照：run20（S=67.186，recovery=N）

---

## 6）结论
1. Map3 具备“系统性不可行”特征：95% 固定超时 + 100% 恢复触发 + 恢复高强度常态化。
2. 关键证据是反例：即使恢复更少（hits=7）仍超时，说明不是恢复策略导致耗尽时间，而是结构性不可行导致无法终止。
3. 后续修复应围绕：全局-局部一致性、局部可行性、目标邻域收敛三条主线推进；修复的唯一有效判据是 Map3 超时率显著下降并能稳定完成导航。
4. “系统性不可行”指：该场景下的 runs 数据不能用于评估/对比算法优劣（例如参数 A/B、map 间对比、live vs replay），因为失败机制呈现稳定、重复、结构化特征，已超出随机噪声或偶发异常的解释范围。