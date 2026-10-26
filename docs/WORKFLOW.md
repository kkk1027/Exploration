# 工作流说明（操作手册）—— 谁接手都按这个跑

## 0. 三条铁律
1. **一次只跑一轮**（`flock /tmp/stemm_round.lock`）；批次也是串行。
2. **改动可回退**：开关默认关，或 `--restore` 逐字节还原；改完立刻 `git commit`。
3. **S0 零接触优先**：出现物理接触即回退该改动，即使别的指标变好。

## 1. 目录约定
| 位置 | 内容 |
| --- | --- |
| `harness/round.sh` | 单轮运行器（起栈/探索/收尾/报告/digest） |
| `harness/batches/run_p0_shots.sh` | 批量：每轮 + 残渣探针 + RViz 每 5 s 截图 |
| `harness/batches/compare_batches.py` | 批次 A/B 对照表（含组间差值） |
| `harness/nav_params.py` | **唯一的** nav2 参数注入器（vx/膨胀/规划器），`--restore` 逐字节还原 |
| `harness/vision_nav_params.py` | 视觉层注入（local voxel / global obstacle） |
| `harness/patch_car_launch.py` | 车端 launch 注入（grace / rrt_wait / front_stop / tf simtime） |
| `harness/nav_sim.sh` | **不用 Gazebo** 的 nav2 台架（fake_base + 离线地图 raycast + audit + RViz） |
| `harness/bench_ab.py` | 在台架上自动跑多配置 × 多目标，输出成功率/计划净宽 |
| `harness/analyze_*.py` | 只读分析（failfast 判据、责任方、真窄族、卡点宽度、候选归因） |
| `harness/test_*.py` | 离线不变量测试（smoke 第 6 组逐条跑） |
| `logs/round_<时间戳>_<tag>/` | 单轮全部产物（launch.log / timeline.jsonl / digest.md / evaluate.json / shots/） |
| `runs/*.jsonl` | 单轮摘要（harness 事件流） |
| `patches/*.pristine.*` | 注入前的原厂快照（**唯一真源**，勿删） |

## 2. 跑一轮（手工）
```bash
cd /home/kbf/stemm_exploration_sim_ws
PASS_W=1.05 VISION_DECAY=20 RECOVERY_BT=replan VISION=1 RVIZ=false GUI=false ROUND_MARGIN=15 \
  bash harness/round.sh <tag> 60          # 参数：<tag> <探索秒数>
```
- 产物：`logs/round_*_<tag>/`；`digest.md` 给出 S0~S3 与卡死清单；`trajectory.png` 带每秒时间戳。
- 起栈约 90 s（Gazebo 路线），探索后按 `ROUND_MARGIN` 等返航。

## 3. 跑一批（正式 A/B）
```bash
export PASS_W=1.05 VISION_DECAY=20 RECOVERY_BT=replan   # 与基线保持完全一致，只改被测开关
export <被测开关>=1
bash harness/batches/run_p0_shots.sh <前缀> 5           # 5 轮 180 s
python3 harness/batches/compare_batches.py nosp <前缀>   # 与基线对照（基线前缀 nosp）
```
- 批次每轮前自动清场：`stack_kill` → 删 `/tmp/stemm_round.lock` → 等 `gzserver` 归零 → 清 `/dev/shm`。
- 每轮另起残渣探针（`costmap_residue_probe.py`）与 RViz 5 s 截图。
- 单批 5 轮 ≈ 23~25 分钟。

## 4. 不用 Gazebo 的快测（首选，改 nav2 参数时用它）
```bash
bash harness/nav_sim.sh                      # 调优配置
NAV_ARGS_OFF=1 bash harness/nav_sim.sh       # 原厂参数对照
python3 harness/bench_ab.py --configs orig,tuned --timeout 35
```
- `fake_base` 吃 `/cmd_vel` 做运动学积分，并用离线地图 raycast 出 `/scan`（含 110°~240° 盲区），
  发 `/odom_combined` + TF；起栈约 10 s，无物理引擎、无接触噪声。
- `nav_plan_audit` 每条 `/plan` 打一行"最小净宽 / 最长窄段 / 是否穿窄处"，可在 RViz 看红点标记。

## 5. 看数（判据口径固定，不要临时改）
| 指标 | 来源 | 说明 |
| --- | --- | --- |
| S0 零物理接触 | `evaluate.json → collision.physical_contacts` | **最高优先** |
| S1 零"卡住" | `digest.md` | cmd≠0 且 ≥3 s 无位移 |
| 最长原地挪动 | `digest.md` 卡死清单（**只取"有活跃目标"的行**） | 空等类会污染，必须剔除 |
| ≥30 s 长停滞次数 | 同上 | 比"每轮事件数"稳健 |
| 覆盖格 | `compare_batches.py` | 0.5 m 格覆盖数，衡量供给是否被掐太狠 |
| 派发来源构成 | `digest.md` | rrt / manager / home |
| 计划净宽 | `nav_plan_audit` / `plan_audit.jsonl` | 只在台架或开启 `PLAN_GUARD_LOG` 时有 |

判定：**任一硬指标退化即回退**；判"保留"必须同时满足"目标指标改善 + S0 不变 + 覆盖格下降 ≤15%"。

## 6. 改参数/注入的正确姿势
```bash
python3 harness/nav_params.py --check                 # 看当前注入状态
python3 harness/nav_params.py --apply --vx-max 0.35 --allow-unknown false --motion REEDS_SHEPP
python3 harness/nav_params.py --restore               # 逐字节还原
```
- 不要再用 `patch_nav_speed.py / patch_costmap_inflation.py / patch_planner.py`（已被 `nav_params.py` 取代，会互相覆盖）。
- 布尔型开关**必须传 `true/false`**（传 `1` 会 int/bool 类型不匹配把节点打崩）。
- 新增可执行文件后要 `colcon build --packages-select stemm_cartographer_exploration_sim`。

## 7. 提交前自检
```bash
python3 harness/test_arbiter_attrs.py && python3 harness/test_arbiter_static.py \
 && python3 harness/test_param_types.py && python3 harness/test_driver_hooks.py \
 && python3 harness/test_launch_dryrun.py && python3 harness/test_harness_setu.py
bash -n harness/round.sh && bash -n harness/batches/run_p0_shots.sh
git status --porcelain          # 确认没有残留注入（sim_patch / vision_fusion_patch 计数为 0）
```
提交信息模板：`fix/feat(<层>): 做了什么 —— 为什么(带数据) —— 开关与回退方式`。

## 8. 收工检查
```bash
bash harness/stack_kill.sh term      # 整栈按可执行路径枚举清理，校验 0 残渣
python3 harness/nav_params.py --restore
python3 harness/vision_nav_params.py --restore
rm -f /dev/shm/fastrtps_* /tmp/stemm_round.lock
```

## 9. 已知环境陷阱（详见 HANDOFF_DAC.md §7）
ros2cli daemon 污染、`ROS_LOCALHOST_ONLY=1`、空 launch 参数、`bash -c` 行内注释、`pkill` 自伤、
timeline 环境变量兜底、PGM 行号翻转、批次切换清场。
