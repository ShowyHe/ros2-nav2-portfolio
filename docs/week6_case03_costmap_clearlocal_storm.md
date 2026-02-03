# Week6 Case03 — Map2 Costmap / Perception（ClearLocalCostmap 高频：表征不稳定/逼近恢复边缘）

## 0）Case 定位
1. Case 名称：Costmap / Perception（ClearLocalCostmap 高频触发）
2. 目标：用 Map2（并对照 Map1）runs 数据证明“代价地图表征经常把系统逼到恢复边缘”，即便成功也并不顺滑
3. 数据范围：
   - Week5 Map2 共 20 runs
   - Week5 Map1 共 20 runs（作为稳定对照）

---

## 1）现象
1. Map2 中 ClearLocalCostmap 触发极其普遍，成功样本里也高频出现。
2. Map2 的导航表现呈现“反复清局部代价地图后继续跑”的模式，成功并不代表过程稳定。
3. Map1 也会触发 ClearLocalCostmap，但频率显著低于 Map2，且恢复强度稳定、轻量。

---

## 2）证据
### 2.1 Map2：ClearLocalCostmap 的出现频次与形态
1. Map2 recovery=Y：16/20（80%）
2. Map2 `ClearLocalCostmap` 出现：16/20（80%）
3. Map2 中大量成功样本呈现同一形态：`ClearLocalCostmap` 且 `hits=2`
   - 典型样本（均为 S）：run2（hits=2）、run5（hits=2）、run6（hits=2）、run7（hits=2）、run10（hits=2）、run14（hits=2）、run15（hits=2）、run18（hits=2）
4. Map2 的失败/超时样本中，ClearLocalCostmap 常与其他恢复叠加，且 hits 明显增大
   - run4：T,120，types=BackUp;ClearGlobal;ClearLocal;Spin，hits=9
   - run8：T,120，types=BackUp;ClearGlobal;ClearLocal;Spin，hits=11
   - run12：T,114.499，types=ClearLocal;BackUp;ClearGlobal;Spin，hits=16
   - run9：F,120，types=BackUp;ClearGlobal;ClearLocal;Spin，hits=31

### 2.2 Map1（对照）：ClearLocalCostmap 触发频率与强度显著更低
1. Map1 结果：20/20 成功（100%）
2. Map1 `ClearLocalCostmap` 出现：6/20（30%）
   - 触发 runs：run7/run9/run12/run15/run18/run20
3. Map1 中一旦触发，恢复强度稳定且轻量：全部 `hits=2`

### 2.3 对照结论（只基于数据）
1. Map2 的 ClearLocalCostmap 触发频率（80%）显著高于 Map1（30%）。
2. Map2 不仅“更常清”，而且在失败/超时样本中容易升级为“多恢复叠加 + 高 hits”。

---

## 3）根因判断
1. 结论级判断：Map2 存在“局部代价地图表征不稳定/局部可行性经常被否决”的症状，ClearLocalCostmap 高频触发是该症状的核心外显信号。
2. 依据：
   1) ClearLocalCostmap 在 Map2 成功样本中高频出现，且形态高度重复（hits=2），说明系统经常处于“接近不可行/需要清理局部代价地图才能继续”的边缘状态。
   2) Map2 的失败/超时样本中，ClearLocalCostmap 常与 BackUp/Spin/ClearGlobal 叠加并伴随 hits 激增，表明局部表征问题可能进一步放大到控制层与全局层的恢复链循环。
   3) Map1 作为对照：同样算法与口径下，ClearLocalCostmap 触发显著更少且强度固定（hits=2），说明 Map2 的高频清理与场景几何/障碍布局强相关，具有场景结构性。
3. 因果链：
   - `ClearLocalCostmap 高频（尤其成功样本也频繁）` → `局部 costmap 常把路径逼到恢复边缘` → `过程不顺滑、鲁棒性不足`
   - 当叠加 `BackUp/Spin/ClearGlobal + hits 激增` 时，表明局部表征问题开始诱发系统性循环风险。

---

## 4）修复动作清单与验证口径
### 4.1 验证口径（先固定）
1. timeout 固定：120s
2. 成功判据：沿用仓库统一口径（Week1 固定的 success_radius）
3. 对照原则：同一 Map2、同一起终点、同一启动流程；只做最小改动并做对照 runs

### 4.2 修复动作清单（按优先级）
1. 局部 costmap 表征核查（先确认“是不是被封路/被污染”）
   - 核查点：obstacle 层与 inflation 层在窄处是否把可通行区域压没；障碍更新是否抖动导致频繁清理。
2. 清理触发条件核查（确认 ClearLocalCostmap 为什么被触发）
   - 核查点：触发点是否集中在特定区域/特定几何结构；是否与传感器噪声/时间同步问题相关。
3. 与控制层耦合核查（确认“清理后为什么仍会升级为 Spin/BackUp”）
   - 核查点：清理发生后局部轨迹是否仍不可行；是否立刻转入 BackUp/Spin 循环。
4. 只做“必要的参数/配置改动”并严格对照
   - 目标不是“减少清理次数本身”，而是减少“清理→升级为多恢复叠加→超时/失败”的链路。

### 4.3 验证方式（最小对照）
1. Map2 baseline 组与修复组各 20 runs（或先 10 runs 快速试验）：
   - 指标：success_rate、median_time、ClearLocalCostmap 出现频次（按 run 计数）、recovery_hits（中位数/均值）
2. 验收条件（量化）：
   - ClearLocalCostmap 触发频次下降（80% → 更低）
   - 超时/失败样本下降（尤其 run9 类高 hits 循环不再出现）

---

## 5）复现入口
1. 数据入口：Week5 Map2 / Map1 runs 记录（本文件引用的 runs）
2. 关键锚点 runs：
   - Map2（成功但高频清理）：run2/run5/run6/run7/run10/run14/run15/run18（hits=2）
   - Map2（清理升级为多恢复叠加）：run4/run8/run12（T）与 run9（F, hits=31）
   - Map1（稳定对照清理）：run7/run9/run12/run15/run18/run20（均 hits=2）

---

## 6）结论
1. Map2 的 ClearLocalCostmap 触发频率（80%）显著高于 Map1（30%），且成功样本中大量重复出现（hits=2），说明 Map2 导航并非顺滑稳定，而是频繁在“局部 costmap 边缘不可行”状态下运行。
2. 当 ClearLocalCostmap 与 BackUp/Spin/ClearGlobal 叠加并伴随 hits 激增时（run4/run8/run12/run9），系统进入高风险循环，最终导致超时或失败。
3. 后续修复应优先围绕：局部 costmap 是否封路/抖动、清理触发机制、与控制层耦合三条主线推进；修复有效性必须以 Map2 的清理频次下降与超时/失败显著减少为唯一验收标准。
