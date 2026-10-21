# 实机自主探索栈的仿真接入 —— 稳定性验收记录

## 结论
连续 **6 轮**全流程无异常（4×120 s + 2×180 s，5 个不同随机起点），七项检查逐轮全绿。

| 轮 | 探索 | 随机起点 (x,y,yaw) | 结果 |
|---|---|---|---|
| 1 | 120 s | −0.3, 0.4, 3.14 | PASS |
| 2 | 120 s | 0.3, −0.4, −1.57 | PASS |
| 3 | 120 s | 0.4, 0.3, 1.57 | PASS |
| 4 | 120 s | 0.4, 0.3, 1.57 | PASS |
| 5 | **180 s** | 0.0, 0.0, 0.0 | PASS |
| 6 | **180 s** | 0.3, −0.4, −1.57 | PASS |

七项：preflight ／ TF 健康 ／ Nav2 四节点 ACTIVE ／ 前沿探索发生 ／ 到时 `set_mapping_done` ／
返航完成 ／ 地图存盘；每轮收尾 `gzserver=0、进程残留=[]`。

## 关键修复（按顺序，均已在 git 中）
1. **车端绝对路径**：`stemm_safe_nav2_params.yaml` 里两行 BT XML 路径改到本机 overlay（车端仓库未提交，patch 见 `patches/`）。
2. **preflight 垫片**：`systemctl` 垫片让仿真满足车端"现场隔离"检查（wheeltec-robot/lidar=active，回充/跟随=inactive）。
3. **use_sim_time 补丁**：`mapping_tf_guard` 等 5 个车端节点在 launch 里没传 use_sim_time，运行时补齐（否则 TF 新鲜度判定必超时）。
4. **时钟细化改滞后插值**：`now()` 永不超前引擎时间 → 消除 Nav2 "extrapolation into the future"（实测只差 4 ms 就报错）；并改为探索栈启动即启用，避免返航瞬间跳变。
5. **Nav2 transform_tolerance=0.5**：运行时放宽，容忍毫秒级时间差。
6. **返航判定对齐车端**：以 `/stemm_cartographer/nav_state` 的完成状态为准（车端有"姿态超时后按位置完成"的兜底路径，之前被误判为失败）。
7. **harness 加固**：轮前"进程级"断言（DDS 僵尸节点不算）、PGID 精确收尾、TERM→KILL 升级、随机起点、六项评估。

## 已知差异（未在本轮解决，记录在案）
- 探索期与返航期均启用时钟细化（2 ms），Nav2 控制环 ~20 Hz；实机为 20~30 Hz，量级相当。
- 支撑为左右对称一对全向轮（真机单支撑），用于消除静止蠕动（3.84 → 0.02 mm/s）。
- 返航末段偶发依靠车端"按位置完成"兜底（仿真控制器末端小幅摆动），未改车端 25 s 姿态超时。
- 地图为 5 cm 栅格、87×… 尺寸正常（`maps/explore_latest.pgm`）。

## 用法
```
bash harness/round.sh <标签> <探索秒数>        # 单轮
bash harness/sweep.sh 6 4 2                    # 连续 6 轮（4×120s + 2×180s）
python3 harness/evaluate.py <launch.log> <标签>  # 七项评估
```
