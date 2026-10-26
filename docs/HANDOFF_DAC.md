# 交接：D→A→C 卡死治理（2026-10-26/27）

## 0. 一句话状态
D 第一版已实现并**因 S0 退化回退**（保留开关）；A 只有规格；C 未做。基线口径与全部工程纪律见下。

## 1. 环境（不在代码里、但新人必知）
- Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11.10；`ROS_DOMAIN_ID=42`。
- 三个工作区：
  - `/home/kbf/stemm_exploration_sim_ws` ← **本仓库**（harness + 探索侧节点/launch）
  - `/home/kbf/stemm_sim_ws` ← 机器人模型/世界/`domain_guard.sh`/EKF 配置
  - `/home/kbf/stemm_explore_ws` ← 车端栈 install（源在 `/home/kbf/wheeltec_ros2`，symlink 编译）
- **`ROS_LOCALHOST_ONLY=1` 必须**（手机热点下组播不稳）；`round.sh` 已强制。
- **ros2cli daemon 陷阱**：daemon 若在没有 `ROS_LOCALHOST_ONLY=1` 的环境里起过，`ros2 node list` 空、
  `ros2 param set/dump` 全超时（而 rclpy 节点间一切正常）。每轮开跑前 `ros2 daemon stop`，否则会出现
  "参数没落地/TF 断了"的假象。
- 包是 `--symlink-install`：**改已有脚本/launch 即时生效**；**新增可执行文件要 `colcon build --packages-select
  stemm_cartographer_exploration_sim`** 才会进 `install/`。

## 2. 工程纪律（用户明确要求，违反会被打回）
1. 导航参数**固定**，不按场景调；调参必须有数据。
2. 一切改动**可回退**：哨兵注释/journal/`--restore` 逐字节还原，或单开关（默认关）。
3. **S0（零物理接触）优先于一切**；出现接触即回退，即使卡死数变好。
4. 每改动 **5 轮 180 s** 同口径对比；退化立即回退并继续下一层；全程 git 提交。
5. 返航失败只记录不解决。
6. 报告只写有证据的数字；不确定就写"未验证"。

## 3. 怎么跑与怎么看数
```bash
bash harness/batches/run_p0_shots.sh <前缀> 5      # 每轮 180s 探索 + 探针 + RViz 截图
python3 harness/batches/compare_batches.py <基线前缀> <实验前缀>
python3 harness/analyze_failfast.py --glob 'logs/round_*'   # L1 判据离线复算（命中/误报/覆盖）
python3 harness/analyze_rootcause.py                        # 责任方统计（车端闸/nav2/控制）
python3 harness/analyze_narrow_family.py                    # 真窄族 vs 接触的关系
python3 harness/analyze_stuck_clearance.py --glob 'logs/round_*'  # 卡点自由宽度分布
```
单轮手动：`PASS_W=1.05 VISION_DECAY=20 RECOVERY_BT=replan VISION=1 RVIZ=false GUI=false ROUND_MARGIN=15 \
  bash harness/round.sh <tag> 60`
**必须单轮串行**（`flock /tmp/stemm_round.lock`）；批次每轮前会清场（stack_kill + 删锁 + 等 gzserver 归零）。

## 4. 现有关键参数状态（默认值）
| 项 | 值 | 说明 |
| --- | --- | --- |
| `PASS_W` | 1.05 | 候选闸门"通行判据" |
| `VISION` / `VISION_DECAY` | 1 / 20 s | 视觉层 live+clearing+decay |
| `RECOVERY_BT` | replan | 重规划 1.0 Hz、**Spin 保持关闭**（会撞墙） |
| `RWG_SEC` | 20 s | 限时到点若有活跃候选，等它结束再返航 |
| `FAILFAST` | 0（关） | L1/L2 判"过不去"→取消+拉黑；实测打不中真窄族，且放宽档出现接触 |
| `WIDTH_UNK_BLOCK` | 0（关） | 判宽时未知按障碍；开=**2259 次接触**，禁用该组合 |
| `UNKNOWN_BLOCK` | 0（关） | 与 `WIDTH_UNK_BLOCK` 同时开会爆接触 |
| MPPI `vx_min` | 0.0 | 受限倒车曾试 -0.08：S0 3/5 → 已回退 |
| Smac `cost_penalty` | 2.5（原厂，未动） | 曾试 8.0 属未批准改动，已回退 |
| footprint_padding | 0.10（注入件） | 内切 0.235→0.335；`PAD=0.29` 会让整图不可通行 |
| `allow_unknown` / `motion_model` | false / REEDS_SHEPP（经 `nav_params.py` 注入） | 用户已批准 |
> 注入统一走 `harness/nav_params.py`（唯一真源 `patches/stemm_safe_nav2_params.pristine.yaml`）；
> 旧的 `patch_nav_speed/patch_costmap_inflation/patch_planner` 已被它取代，勿再用。

## 5. 已确认的事实（可直接引用）
- 基线 `nosp`（5 轮）：卡死 3.80/轮、卡住 2.20/轮、**有活跃目标的最长原地挪动 24 s**、接触 0/5、覆盖 63.6 格。
- 全库 534 轮：**有物理接触 45 轮（8%）**；接触区宽度 <0.47 m 占 **24%**、0.47~1.05 m 27%、≥1.05 m 49%。
- 真窄卡死族：**19 次 / 14 轮**（每轮 ≤3），热点 `(-9,-1)`（窄点×3、接触×7）、`(-8.5,-1)`、`(-8.5,-2)`。
- 卡死事件 181 个：78% <10 s、5.5% ≥30 s；`nav` 众数 executing 29%、waiting_for_rrt_frontier 29%、
  blocked_waiting_clearance 11%；68% 的卡点在轮末地图上 ≥1.05 m（"看着宽"）。
- **机制链（真窄族）**：地图/盲区让它"看起来能过" → 进入 → front<0.65 → 车端安全闸零速 →
  `vx_min=0` + Wait 恢复 + 前向规划 ⇒ **无退出动作，吸收态** → 期间接触。
- 责任划分：**nav2+感知负责"送进去"，车端负责"出不来"**；只补事后能力（L1/L2）无法降低发生次数。

## 6. D/A/C 三层（现状与下一步）
- **D 第一版已完成并回退**：`width_unknown_block`（提交 `1a1181d`）+ `unknown_block` 布尔化修复（`c578315`）。
  冒烟 60 s 出现 **2259 次接触** → 见 `docs/P9` §12；可试的克制版：①只改距离场不动可达性；②只对车周 2 m；
  ③未知按"半宽"折算。
- **A 只有规格**（`docs/P9` §13，提交 `9afae6f`）：`noexit_enabled`（默认关）、`pinch=0.752 m`、
  `short=0.35 m`、`retreat=1.2 m`、`turn=0.81 m`；三条判据任一成立即放行，否则硬否决并计 `reject_noexit`。
  实现位置：`frontier_arbiter::path_and_width` 逐格采样之后。**注意**：A 依赖宽度输入正确（D），当前 D 回退中，
  所以 A 只能拦"已看清的窄处"。
- **C 未做**：`escape_supervisor` 里 `explore_reverse_enabled` 已写好（雷达复核后方净空、单步 ≤0.15 m @0.06 m/s、
  默认关），但从未启用验证。这是"已进入后能否退出"的唯一希望。

## 7. 踩过的坑（省下重复时间）
1. **布尔型 launch 参数**：`X=1` 会被解析成 int → 节点 `declare(bool)` 抛 `InvalidParameterTypeException`
   当场崩（表现是"起栈失败/无 timeline"）。已修 `failfast/plan_guard/unknown_block/supervisor_explore_reverse`。
2. **空 launch 参数值**：`foo:=` 空串 → launch 报 `malformed launch argument`。`vision_dump` 已强制非空。
3. **`bash -c "..."` 里不能写行内 `#` 注释**（会变成参数值的一部分）。
4. **pkill 自伤**：模式若出现在自己命令行（含"启动同一脚本"的尾句）会把自身杀掉——本会话踩了 3 次。
5. **timeline 采不到**：launch 参数偶发丢失 → 仲裁器改读环境变量 `STEMM_TIMELINE_LOG`（`round.sh` 导出）。
6. **探针两个真 bug**：`set -u` 下 `source setup.bash` 让子 shell 静默退出；`costmap_raw` 是
   `nav2_msgs/Costmap` 不是 `OccupancyGrid`。都已修，并有 `test_harness_setu.py` 防回归。
7. **y 轴镜像**：PGM 行 0 在图上方、origin 在左下，行号必须 `h-1-(y-oy)/res`（`fake_base`/`nav_plan_audit` 已修）。
8. **批次切换**：上一批刚收尾就起新批会拿不到轮目录 → 已加清场（stack_kill + 删锁 + 等 gzserver 归零）。

## 8. 推荐顺序（给下一个智能体）
1. C 先行（成本最低、唯一能打破吸收态）：启用 `explore_reverse_enabled`，1 轮冒烟看 `reverse_assist_start`；
   5 轮对照，主看接触是否仍为 0、≥30 s 长停滞是否下降。
2. A 实现（按 §13），先 1 轮冒烟确认 `reject_noexit>0` 且接触 0，再 5 轮。
3. D 克制版（§12.3 三选一）最后做，因为它的历史副作用最大。
4. 全部完成后把 §12/§13 与实测数据合成 `docs/P9` 最终总结。
