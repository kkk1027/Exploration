# P0 基线冻结（可入库副本）

## 为什么这些文件必须在仓库里

`logs/` 与 `runs/` 都在 `.gitignore` 里，所有批次数据只存在于本机。
这带来两个具体问题：

1. **换机器或清理磁盘后，基线就没了**。P0 的结论
   （`frame_far_first` 收益不显著、返航失败有两种死锁）是后续所有对照的起点，
   不能依赖"某次本地 logs 里还有"。
2. **别人无法复核**。把数据冻结成 jsonl 后，任何人都能重算中位数、
   配对差值与符号检验，而不是只能相信文档里的数字。

生成方式：`python3 harness/freeze_baseline.py <tag...> -o harness/baselines/<name>.jsonl`
（脚本从 `logs/` 读取，缺失字段写 `null` 而不是 `0`，便于区分"没测到"与"测到 0"）。

## 文件清单

| 文件 | 批次 | 轮数 | 出生点 | 用途 |
|---|---|---|---|---|
| `p4_pre_p0.jsonl` | `p4base` × 10 + `p4far` × 10 | 20 | **随机**（5 个候选中随机） | **P0 改造前的历史基线** |
| `p14vS_p0_smoke.jsonl` | `p14vS_far` × 4 + `p14vS_base` × 4 | 8 | 固定单一 `1.3220 3.9987 -1.5708` | P0 中间冒烟 |
| `p14vU_p0_paired.jsonl` | `p14vU_far` × 10 + `p14vU_base` × 10 | 20 | 固定 5 点 × 各 2 轮，**同点 far/base 交替** | **P0 正式配对（结论来源）** |

## 各组速查

| 批次 | 组 | 接触合计 | 返航成功 | `known` 末值中位 |
|---|---|---|---|---|
| `p4_pre_p0` | far | **1** | 2/10 | 65,879 |
| `p4_pre_p0` | base | **323** | 3/10 | 65,229 |
| `p14vS_p0_smoke` | far | 0 | 4/4 | 65,704 |
| `p14vS_p0_smoke` | base | 0 | 4/4 | 65,474 |
| `p14vU_p0_paired` | far | **16** | 9/10 | 66,933 |
| `p14vU_p0_paired` | base | **31** | 9/10 | 66,342 |

## 三个必须知道的坑

### 1. `p4_pre_p0` 的"接触 323"不能当基线用

323 次接触里 **313 集中在同一轮**（`round_20261111-203418_p4base`），
组均值被单轮灾难拉爆。而且两组用的是**随机出生点，非配对**，
组间差异混杂了出生点差异。

**所以 P0 正式结论只取 `p14vU`（固定出生点、同点交替、配对检验）。**
`p4_pre_p0` 保留的价值是"改造前是什么样"的历史留档，不是对照基准。

### 2. 组中位数之差 ≠ 配对差值中位数

`p14vU` 的两个数字容易混淆：

* **组中位数之差** = 66,933 − 66,342 = **+591**
* **配对差值中位数** = median(far_i − base_i) = **+157**

前者把"不同出生点之间的差异"也算进去了；后者在同一出生点内比较，**才是有意义的**。
判定用配对值（`harness/p0_paired.py` 的输出）。

### 3. `known` 末值不等于覆盖率

`known` 是"已知格数（空闲 + 占据）"，会随地图尺寸变化（本轮地图
233–236 × 385–387 @ 5 cm）。跨批次比较时只能看**同批次内的相对差异**，
不要拿绝对值和别的世界比。

## 复核方法

```bash
# 配对分析（结论来源）
python3 harness/p0_paired.py --pairs /tmp/p14vU_pairs.txt --pattern p14vU

# 单独看冻结数据
python3 - <<'PY'
import json, statistics as st
rows = [json.loads(l) for l in open('harness/baselines/p14vU_p0_paired.jsonl')]
far = [r for r in rows if r['group'] == 'far']
base = [r for r in rows if r['group'] == 'base']
print('far 接触', sum(r['contacts'] for r in far),
      'base 接触', sum(r['contacts'] for r in base))
print('far known 中位', st.median([r['known1'] for r in far if r['known1']]))
PY
```

---

## `p14x3_cov_terminate.jsonl`（2026-11-15，4 轮）

**用途**：S5 P2 覆盖率终止 + 视觉盲区修正 + 走廊分级落地后的**当前基线**。
替换 `p14vS_p0_smoke` 作为日常回归对照。

| 项 | 值 |
|---|---|
| 轮数 | 4（4 个固定出生点，每轮 900 s 上限） |
| 物理接触合计 | **0** |
| 返航成功 | **4/4** |
| `known` 末中位 | 66,173 |

**该批次时的生效配置**（与冻结数据一一对应，复现时必须一致）：

| 参数 | 值 | 所在位置 |
|---|---|---|
| `coverage_terminate` | true | `round.sh` 默认 `COV_TERM=1` |
| `coverage_threshold` | 0.97 | manager 默认（未改） |
| `coverage_min_explore_sec` | 150.0 | `stemm_cartographer_auto_mapping.launch.py` |
| `coverage_min_known_cells` | 48500 | 同上 |
| `vision range_min` | **0.0** | `vision_obstacle_grid` 默认 |
| `vision_narrow_soft` | true | `frontier_arbiter` 默认 |
| `approach_corridor_hard_occupancy` | 65 | `cartographer_manager` 默认 |
| `approach_corridor_gray_frac_max` | 0.34 | 同上 |
| `approach_probe_m` | 0.2 | launch |

**重要局限（必须连同基线一起引用）**：

1. **走廊"拒绝→降级"路径在这 4 轮中未被触发**。4 轮返航里只有 1 轮出现
   1 次重试，原因是 `return-home goal failed with status 6`（规划失败），
   **不是**走廊拒绝。所以本基线只能证明走廊分级改动
   **没有引入回归**，**不能**证明它修好了原缺陷。
   要验证修复效果，需要构造"起点紧贴模糊占用格"的复现场景。
2. `known` 末中位 66,173 与 `p14vS_p0_smoke` 的 65,704 同量级；
   但本批启用了覆盖率终止（150 s 起可能提前结束），
   与定时器批次的 `known` **不可直接比较**。
3. 只测了 `aws_small_house.world`；`coverage_min_known_cells=48500`
   与本房型尺度绑定，换图必须按新图 `known` 收敛值 ≈98% 重算。
