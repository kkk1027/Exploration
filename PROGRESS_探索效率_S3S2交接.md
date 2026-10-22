# 探索建图效率改进（S0/S1/S2/S3）进度交接

> 目的：把 2026-10-22 这一轮"探索效率提升"做到的状态、参数、实测数据和后续方向固化下来，
> 便于在 P0（脱困改造）之后继续这条线。所有改动都在 `stemm_exploration_sim_ws`（仿真侧），
> 车端 `wheeltec_ros2` 源码零提交（仅一个 sim 路径 patch 文件）。

## 1. 一句话现状

实机探索栈（RRT 前沿链 → Nav2）在仿真里 **能稳定跑完 6 轮全流程**（sweep5：6/6 PASS，
含 2×180 s、随机起点、零残留）；效率侧已落地 S1（可达性预筛 + 黑名单 + 最小前沿 + 回拉）、
S2（Nav2 空闲即补候选，`republish_sec=1.5`）、S3（受阻区记忆降权）。
**未解决**：车体撞障（P0 正在做）、脱困动作低效（旋转到底但代价不改善）、
"选点偏向已探完区域"的收益函数细化。

## 2. 代码位置

| 文件 | 作用 |
| --- | --- |
| `src/stemm_cartographer_exploration_sim/scripts/frontier_arbiter` | S1/S2/S3 全部逻辑（可达性 BFS、黑名单、受阻区降权、空闲补发） |
| `src/stemm_cartographer_exploration_sim/scripts/patch_sim_time` | 运行时参数补丁（use_sim_time、frontier_goal_topic、transform_tolerance、目标容差） |
| `src/stemm_cartographer_exploration_sim/launch/sim_explore.launch.py` | 三层组装（机器人 / 车端栈 / 会话驱动）+ 仲裁节点 |
| `harness/round.sh` | 单轮全流程（启动前断言、随机起点、PGID 收尾、零残留取证） |
| `harness/evaluate.py` | 七项验收（preflight/tf/nav2/frontier/mapping_done/return_home/map_saved） |
| `harness/time_breakdown.py` | 时间占用分布（启动 / 导航 / 选点等待 / 恢复 / 返航） |

## 3. 当前的调参面（生效值）

仲裁 `frontier_arbiter`（launch 里传的就是这些）：
```
republish_sec   1.5      # S2：Nav2 没有活动目标时立刻补候选
max_candidates  6        # 每轮校验候选数
blacklist_sec   30.0     # 失败候选（路径不可达/被拒）黑名单时长
blacklist_radius 0.5
min_unknown_cells 30     # 前沿规模下限（5 cm 栅格）
check_radius    1.0      # 统计未知格的半径
pullback_m      0.6      # 目标回拉到已知自由侧
# 内部：黑名单=扣 250 未知格收益（重罚不硬拒）；受阻区=1.5 m 内扣 400，最多 15 个，60 s 衰减
```

车端 manager 运行期被改的参数（`patch_sim_time`）：
`frontier_goal_topic=/validated_frontier_goal`、五个节点 `use_sim_time=true`、
Nav2 六端点 `transform_tolerance=0.5`、`controller_server` 目标容差 0.15/0.15。

## 4. 实测数据（同口径）

| 轮次 | 派发/成功/取消 | 导航时间占比 | 选点等待 | 恢复 | 返航 | 结论 |
| --- | --- | --- | --- | --- | --- | --- |
| S1 基线（`s1b2`） | 6 / 4 / 2 | 78.7% | 21.3% | 36% | 到位 | 选点等待长 |
| S2（`s2r1`） | 6 / 4 / 2 | 82.0% | 8.6% | 52% | 到位 | 选点降下来了，恢复变长 |
| S3 v1（`s3r2`） | 5 / 2 / 2 | 148.9% | — | 103% | 失败 | 受阻区吃太狠（314 个） |
| S3 v2（`s3r3`） | 5 / 2 / 2 | 148.9% | — | 103% | 失败 | 事件判定收紧后仍差 |
| 三项修复（`s3fix1`） | 5 / 4 / 1 | 93.9% | 6.1% | — | 真实 23.9 s | **当前最好**，7/7 PASS |
| 演示（`live_144256`，带 rviz2） | 7 / 5 / 2 | 探索 174.6 s | — | — | 未完成 | 撞障集中在 (2.3,3.44)/(3.07,3.65) |

已知结论：
* 候选产量从来不是瓶颈（75 s 内 163 个），瓶颈是"派发出去能不能到"。
* 恢复时间几乎全部来自"楔死 → 盲退 0.30 m → 再楔死"的循环（见 P0 记录）。
* `time_breakdown.py` 的百分比以"探索窗口"为分母，>100% 说明窗口内多段并行/重复计时，只用于相对比较。

## 4b. S3+S2 第二迭代实测（2026-10-23 晚，含 A/B）

收益函数三版对比（同一套 harness；物理接触 = Gazebo 底盘/轮接触传感器，已过滤地面）：

| 版本 | 轮次 | 结论 | 物理接触 | 选点等待 | 派发/成/败/取 | 返航 |
| --- | --- | --- | --- | --- | --- | --- |
| v1（旧：未知格数+就近） | v1r | FAIL | 0 | 33.3% | 6/3/0/1 | 失败 |
| v2（邻域未知占比） | s3a | FAIL | **412（轮）** | — | 6/3/0/2 | 失败 |
| v2 | s3b | FAIL | **266（底盘 249）** | — | 3/0/3/0 | 失败 |
| v3（朝前未知锥+贴已知边界） | v3a | FAIL | 0 | 49.4% | 6/4/0/0 | 失败 |
| v3 | v3b | PASS | 0 | 20.8% | 5/1/0/2 | 40.6 s |
| v3+硬转向门 | v3c | PASS | 0 | 49.1% | 4/1/0/1 | 47.0 s |
| v3+硬转向门 | v3d | FAIL | 0 | — | 3/2/0/1 | 失败 |
| **v3+软转向门+返航连续倒车** | **v3e** | **PASS** | **0** | **8.6%** | 5/4/0/0 | **29.8 s** |
| 同上 | **v3f** | **PASS** | **0** | 30.8% | 7/4/1/0 | 79.6 s |

结论：
* **v2 明确退化**（往未测绘深处扎 → 撞上看不见的桌底座），已回退、默认关闭；
* **v3 是正解方向**：8 轮里物理接触全 0，选点等待从 33~49% 压到 8.6~30.8%（最好一轮 9.8 s）；
* 默认值已改为 `scoring:=v3`，需要退回旧逻辑时用 `SCORING=v1`。

**选点等待的根因（日志实证）**：manager 自己有一条"目标需要的转向 > 80° 就拒绝"的门，
被拒后会先做一次 Nav2 Spin 恢复（实测一次 8.8~10 s 空转）。仲裁原来不知道这条门，
会把背后的候选派出去被打回。现在仲裁按同一口径（75°，留 5° 余量）提前筛，
且"全部超限时仍然派发最优"（软门，避免探索停摆）。

**返航卡死**：manager 放弃后，`escape_supervisor` 现在允许"连续小步倒车"
（每步仍 ≤0.15 m、逐步雷达复核后方 ≥0.30 m 净空、最多 4 步），v3e 靠它 29.8 s 完成返航。

## 5. 后续（本线未完成，P0 之后再接）

1. **收益函数细化**：现在 `info_gain = 半径 1.4 m 内未知格数 × 7`，按"未探区域大小"排序，
   但仍会把目标派到"已基本探完只剩下小碎角"的区域。方向：把未知格按连通块大小加权
   （大块未知 ≫ 碎角），并对"该目标点已被探索拉锯过 N 次"的位置降权。
2. **前沿质量**：候选点目前由 RRT 链给出，未被要求"前沿段长度 ≥ L"；可加 `min_frontier_len`。
3. **选点等待**：`s3fix1` 已到 6.1%，若 P0 之后恢复时间下降，可再看是否放宽 `republish_sec`。
4. **相机/视觉融合**：Astra S（640×480、fx=fy=570.34、58.6°×45.7°、26.6/27.5 Hz）尚未接入，
   规划是 `stemm_vision_mapping`（RTAB-Map）与现有 cartographer 并存，等 P0 稳定后再开。

## 6. 如何复跑

```bash
cd /home/kbf/stemm_exploration_sim_ws
colcon build --packages-select stemm_cartographer_exploration_sim && source install/setup.bash
harness/round.sh <标签> 120          # 单轮 120 s
harness/sweep.sh 6 180 120 120        # 多轮
harness/time_breakdown.py logs/round_*/launch.log
```
无 GUI：`rviz:=false` 由 `harness/round.sh` 默认（launch 默认 true，只有演示轮用
`harness/start_explore_gui.sh`）。
