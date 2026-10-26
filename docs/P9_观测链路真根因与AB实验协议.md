# P9 观测链路真根因（2026-10-26）与卡死治理 A/B 实验协议

本文档记录本轮"先把尺子校准，再谈改代码"的完整过程。起因是用户提出的流程批评：
**"纯数据、低上限调试"、"无法得到小车的可视化轨迹信息"**。校准尺子的过程中发现，
此前若干轮"卡死/参数没落地/探针没数据"的结论里，有相当一部分不是车的问题，
而是**观测链路本身坏了**。以下每条都给出了现象、根因、修复和验证方式。

---

## 0. 一句话结论

| # | 假象（我们此前的解读） | 真根因 | 修复 | 验证 |
| --- | --- | --- | --- | --- |
| 1 | "残渣探针跑了但没有残渣" | `set -u` 下 `source setup.bash` 让**子 shell 静默退出** | 子 shell 内 `set +u` | `costmap/residue.jsonl` 从 0 行 → 619 行 |
| 2 | "探针挂上了但 0 采样" | `costmap_raw` 是 **nav2_msgs/Costmap**，不是 OccupancyGrid | 改消息类型 + 两种编码归一化 | 20 s 拿到 30 行 |
| 3 | "TF 断了 / 参数设不上 / node list 空" | **遗留 ros2cli daemon** 在没有 `ROS_LOCALHOST_ONLY=1` 的环境里起过 | 每轮开跑前 `ros2 daemon stop` | `ros2 node list` 从 0 → 30+ 节点 |
| 4 | "参数落地自检 0/8，本轮 invalid" | `ros2 param dump` 依赖 #3 的 daemon；且 `get_parameters` 混入未声明名字会返回空 | 自检改 **rclpy 直连** + `list_parameters` 先求交集 | 0/8 → **8/8，1.0 s** |
| 5 | "已有另一轮仿真在跑"（白丢 2 轮） | flock 的 fd 被 `setsid` 子进程继承，上一轮被 kill 后**锁仍被孤儿持有** | 子进程 `9>&-` 关闭锁 fd | 连续 5 轮无冲突 |
| 6 | "单轮 14 分钟，一批 5 轮要 70 分钟" | `waited=$((waited+10))` 累加，但每拍实际 16 s（sleep 10 + 采样） | 改 `date` 差值计时 + `ROUND_MARGIN` | 470 s 上限 → 实际 470 s |

> 这六条里有 4 条会**直接伪造出用户看到的现象**：
> 探针没数据 → "旋转残渣不存在/无法取证"；daemon 失联 → "TF 断了、参数没落地"；
> 自检假失败 → 每轮被标 invalid；锁被孤儿持有 → 轮次静默丢失。

---

## 1. `set -u` × `source setup.bash`：静默退出

**现象**：批次脚本每轮都挂了残渣探针，`<round>/costmap/` 目录根本不存在，
`costmap_probe.log` 是空文件（连一句报错都没有）。连续 4 个批次（p1/p2/p3/base）
都是 0 采样，而我们此前把"没有 rot_growth 事件"当成了"没有残渣"。

**根因**：`harness/batches/run_p0_shots.sh` 首行 `set -u`；探针挂载用的是子 shell

```bash
( source /opt/ros/humble/setup.bash >/dev/null 2>&1 ; ... ) &
```

ROS 的 `setup.bash` 会展开未绑定变量（`AMENT_TRACE_SETUP_FILES`）。
在 `set -u` 下 bash 报"未绑定的变量"并**整个子 shell 退出**；而那一行正好把 stderr
重定向到了 `/dev/null`，所以外层完全看不到。

**复现**（1 行）：

```bash
bash -c 'set -u; ( source /opt/ros/humble/setup.bash; echo SHOULD_NOT_PRINT ); echo rc=$?'
# rc=1，SHOULD_NOT_PRINT 不出现
```

**修复**：子 shell 内先 `set +u`。**防回归**：`harness/test_harness_setu.py`
静态扫描 `harness/**/*.sh`，凡有 `set -u` 又 `source` ROS setup 且前面没有
`set +u` 的一律报错（已接入 `smoke_check.sh` 第 6 组）。

## 2. `costmap_raw` 的消息类型

**现象**：探针能起、能就绪，但 `residue.jsonl` 一行不写。

**根因**：`/local_costmap/costmap_raw` 发布的是 `nav2_msgs/msg/Costmap`
（`uint8[]`，255=未知、253=致命），而探针按 `nav_msgs/OccupancyGrid`（`int8[]`，-1=未知）订阅
→ 类型不匹配，一条消息都收不到；`tick()` 里 `if self.raw is None: return` 把整轮吞掉。
`ros2 topic info -v` 里"Topic type: nav2_msgs/msg/Costmap"是直接证据。

**修复**：按 `nav2_msgs/msg/Costmap` 订阅；`occ_of()/unk_of()/grid_shape()` 两种编码归一化；
并且**缺 raw 时退化为用膨胀层出数**（宁可少一列，也不要整轮零输出），
首次退化时打 warn —— "静默零数据"比"报错"危险得多。

## 3. 遗留 ros2cli daemon（本轮最贵的一个坑）

**现象**：`ros2 node list` 输出空；`ros2 param set` 逐项"超时"；`ros2 param dump` 全空；
但 rclpy 节点之间订阅/服务完全正常（探针能收到 costmap、TF 正常、导航在跑）。

**根因**：ros2cli 的 daemon 是**常驻进程**。本机切到手机热点（`172.20.10.5/28`）之前的某次
CLI 调用把它起在了**没有 `ROS_LOCALHOST_ONLY=1`** 的环境里；之后 `round.sh` 虽然强制了
回环发现，但所有 CLI 命令都复用这个"看不见回环节点"的旧 daemon。

**证据（同一进程、同一条命令，只差 daemon）**：

```
--- with existing daemon ---      （空）
--- stop daemon ---
--- after daemon restart ---      /probe_node
```

**修复**：`round.sh` 起栈前 `ros2 daemon stop`（让 CLI 用本轮环境另起一个）。
**连锁收益**：`patch_sim_time` 的逐项 `ros2 param set` 立刻开始出现 `✓`
（例：`/stemm_cartographer_mapping_tf_guard use_sim_time=true ✓`）。

## 4. 参数落地自检：从 0/8 假失败到 8/8

**现象**：每轮 `param_audit` 全项 `实际 ''`，被标 `invalid(param_audit)`；
而且旧实现每个节点重试 90 s，一轮白掉 ~4.5 分钟。

**根因（两层）**：① 依赖 #3 的 daemon → `ros2 param dump` 恒空；
② 即使换成服务直连，**把未声明的名字混进 `get_parameters` 请求会让回应整个空掉**
（实测：10 个名字混着未声明项 → `values` 空；只请求已声明项 → 立刻返回 `22.0`）。

**修复**：`harness/param_audit_probe.py`（新）——rclpy 直连，
先 `list_parameters` 拿到**真正声明过**的名字，再求交集取值：

* 硬门槛（错了整轮数据就没意义）：`use_sim_time`（全为 True）、`footprint`（`[[-0.18,-0.235]`）；
* 软门槛（车端注入项）：`return_home_orientation_grace_sec`、`rrt_fallback_wait_sec`、
  `front_stop_distance`、`exploration_retreat_*`；没声明只提示、不判失败。

结果：`参数落地自检：8/8 通过（硬门槛失败 0 项；节点=3，耗时 1.0s）`。
`param_audit.sh` 退化成 12 行的薄壳（`timeout 60` 兜底）。

## 5. flock 被孤儿继承 → 白丢两轮

**现象**：批次里连续两轮报"已有另一轮仿真在跑"直接退出，2 分钟后才自己恢复。

**根因**：`exec 9>"$LOCK"; flock -n 9` 之后，`setsid bash -c "ros2 launch ..."` **继承了 fd 9**。
上一轮被 kill -9 时，如果有子进程还活着，它替我们把锁**继续持有**。

**修复**：所有派生进程追加 `9>&-`；批次侧再包一层
`timeout -k 10 <explore+margin+300> setsid bash round.sh ...`，保证不退出也会被杀。

## 6. 计时口径：`waited += 10` 不等于 10 秒

**现象**：`探索 180 s + 余量 300 s` 的一轮跑了 ~14 分钟，一批 5 轮要 70 分钟。

**根因**：`sleep 10; waited=$((waited+10)); mem_sample`，而 `mem_sample` 里
`ps`/`/proc` 采样平均要 6 s → 每拍实际 16 s，"480 s"实际 770 s。

**修复**：`waited=$(( $(date +%s) - T0 ))`；余量抽成 `ROUND_MARGIN`（本轮 A/B 统一取 90 s，
因为"返航失败只记录不解决"，没必要给 300 s）。

---

## 7. A/B 实验协议（本轮卡死治理）

**假设（全部来自用户提报的现场现象）**

| 编号 | 改动 | 对应现象 |
| --- | --- | --- |
| ① | BT 恢复动作 `<Wait 2>` → `<Spin>`（受限角 0.6 rad / 6 s） | "卡死在规划起点、重规划也出不来" |
| ② | BT `<RateController hz 0.3>` → `1.0` | "换一条路径能不能脱困？"——原来 3.3 s 才重算一次 |
| ④ | 全局视觉层改 live cloud + `clearing: True` + `decay_sec 20` | 家具被挪走后旧障碍永久堵路 |
| ⑤ | `pass_width 0.90 → 1.05` + 脱困目标必须带掉头空间 | "勉强能通的窄处"仍被判可通行 |

**协议（两批必须完全一致）**

* 每批 5 轮，探索 180 s，`ROUND_MARGIN=90`，`POSE_OVERRIDE="0.0 0.0 0.0"`，
  `VISION=1`、`veto=true`、`max_candidates=14`、`vx_max=0.35`；
* 每轮同时挂：残渣探针（0.5 s JSONL）、RViz 窗口截图（5 s）、时间线（1 Hz）；
* 判据（逐级加严）：**S0** 零物理接触、**S1** 零"卡住"（cmd≠0 且 ≥3 s 无位移）、
  **S2** 零看门狗静止超时、**S3** 返航完成（**只记录不阻塞**）；
* 附加指标：卡死事件数/轮、卡点净空宽度分布、派发来源构成、0.5 m 格覆盖数、
  旋转期间障碍格增长（探针 `probe_rot_grew_cells`）。

**对比命令**

```bash
python3 harness/batches/compare_batches.py base after
```

---

## 8. A/B 结果与 ①（BT Spin）的归因回退

### 8.1 三组怎么来的

| 组 | 配置 | 轮数 | 说明 |
| --- | --- | --- | --- |
| `base` | 改动前 | 5 | 恢复动作 `Wait(2)`、重规划 0.3 Hz、全局视觉层累积云、`pass_width 0.90` |
| `after` | ①②③④ 全开 | 5 | 恢复动作改 `Spin(0.6 rad/6 s)`、重规划 1.0 Hz、视觉层 live+clearing+decay 20 s、`pass_width 1.05` + 脱困落点要掉头空间 |
| `nosp` | **只回退 ①**（②③④） | 3→5 | `RECOVERY_BT=replan`：恢复动作回原厂 `Wait(2)`，其余同 `after` |

三组同域、同起点、180 s 探索、`ROUND_MARGIN=90`、VISION=1、探针 + 每 5 s RViz 截图。

### 8.2 结论一：① 用"物理接触"换位移，必须回退

`after` 组的第 5 轮出现 **S0 破坏**：车被顶在 (-9.10,-0.99) 的墙角，`mode=safety_stop`、
`nav=blocked_waiting_clearance`、`front≈0.26 m`，恢复动作给出的原地旋转（cmd 角速度
0.85~1.1 rad/s，与 behavior_server `Running spin` 对应）让车体与墙发生**底盘接触 94 次**，
并且同一位置僵持 **82 s**。

| 指标 | base | after（含 Spin） | nosp（无 Spin） |
| --- | --- | --- | --- |
| 物理接触/轮 | **0** | **18.8**（其中 1 轮 94 次） | **0** |
| S0 零接触 | 5/5 | 4/5 | 3/3 |
| 卡死/轮 | 5.00 | 2.40 | 3.67 |
| 其中卡住(cmd≠0)/轮 | 3.00 | 2.00 | 2.67 |
| S2 零看门狗超时 | 1/5 | 0/5 | 2/3 |

判读：① 确实把"卡死"变少（5.00→2.40），代价是**突破 S0（零物理接触）**这一最高判据；
去掉 Spin 后接触回到 0，卡死仍比基线低（5.00→3.67），S2 反而最好（2/3）。
按既定纪律"任何退化立即回退"：**① 回退，保留 ②（重规划节拍）**。

> 顺带解释了历史现象：原厂 BT 里 Spin 本来就被注释掉过，理由是
> "12 次 spin 失败里 10 次是 Collision Ahead 否决"。behavior_server 的 Spin 只在
> **开始前**做碰撞预检，一旦已经在角落里，预检通过、旋转过程中就蹭上去了。

### 8.3 结论二：②③④ 的净收益（相对 base，Spin 已回退）

- 卡死/轮 5.00 → 3.67（−27%）；其中"想动却动不了" 3.00 → 2.67（−11%）
- S2（零看门狗静止超时）1/5 → 2/3；轮级 verdict 全绿 3/5 → 3/3
- 旋转期障碍格增长 1950 → 929 格/轮（−52%，③ 的 live+clearing+decay 起作用的直接读数）
- 代价：0.5 m 格覆盖 63 → 53（−15%），最远探索半径 8.43 → 7.10 m。
  机制上说得通：`pass_width 1.05` 让 `rejected_narrow` 从 270~373 涨到 587~671（约 2 倍），
  能派的目标更少、更近。

### 8.4 仍未解决

1. **S1（零"卡住"）三组都没过**：最好的一组仍有 2~3 次/轮的 cmd≠0 无位移。
2. 卡住点大多**不在窄处**：base 15 个卡住点里 10 个（67%）落在 ≥0.90 m 净空处，
   nosp 组 10 个里 4 个（40%）落在 ≥1.05 m 处 —— 说明"卡住"主要是
   **接近/对准/安全闸**问题，不是"通道太窄"。这正好指向唯一还没批准的动作：
   **受限倒车 `vx_min`（−0.08 m/s）**。
3. 归因率（派发目标 ↔ 卡死事件的最近邻匹配 ≤4 m）：base 12%、after 14%，基本不变。

---

## 9. 只拉导航栈的现场验证（nav_verify，用户 2026-10-26 指示）

> 用户原话："后续先只拉导航栈和 rviz2，我来验证效果。而不是重跑探索建图，
> 因为和探索建图的关系不大，而是和 nav2 强相关。"

```bash
cd /home/kbf/stemm_exploration_sim_ws
COST_INFL=1 VISION=1 GAZEBO_GUI=true RVIZ=true bash harness/nav_verify.sh
# 对照（原厂代价地图参数）：COST_INFL=0 bash harness/nav_verify.sh
```

起的东西：Gazebo 世界 + 机器人 + 车端 nav2（`auto_explore:=false`，不会自己跑）+ RViz
+ `nav_plan_audit`。**没有**探索/仲裁/脱困监督参与，退出时自动撤销全部注入。

当前注入的调优值（可直接 `ros2 param get` 核对）：

| 参数 | 原厂 | 现在 | 含义 |
| --- | --- | --- | --- |
| `global inflation_layer.inflation_radius` | 0.4 | **1.0** | 1 m 内都算"贵" |
| `global inflation_layer.cost_scaling_factor` | 3.0 | **1.2** | 代价衰减更慢 ⇒ 离障碍远才便宜 |
| `footprint_padding`（两个 costmap） | 无(0.01) | **0.10** | 内切 0.235→0.335 ⇒ 净宽 <0.67 m 直接致命 |
| `local inflation_radius / scaling` | 0.45 / 3.0 | 未动 | 局部跟线仍要灵活 |
| `vx_max` | 0.5 | 0.35 | 冲死角速度 |

验证方法（现场）：

1. RViz 里用 **2D Goal Pose** 给一个目标（最好挑"明明有宽路可选"的点）；
2. 终端立刻出现一行体检结论，例如实测样例：
   `[plan#4] 137 点(跳过起点 11) 最小净宽 0.20 m @(-0.88,-0.82) 窄于判据的连续段 0.85 m
    / 比车还窄的点 6 个 → ✗ 穿窄处（判据 净宽≥1.05、车宽 0.47、窄段≤1.00；累计 2/4 条穿窄处）`
3. RViz 里 `/plan` 是橙色粗线；`/plan_audit/markers` 把窄处点画成红点、整条计划最窄点画成球
   （黄=仅窄于判据，橙=比车还窄）。
4. 把目标稍微挪一点再给一次，比较两次的最小净宽 —— 可以直接看出"换条路会不会更宽"。

日志：`logs/nav_verify_<HHMMSS>/launch.log`、`plan_audit.jsonl`（每次计划一行结构化记录）。

起栈踩过的三个坑（已在 launch 里修）：

* 缺 `sim_shims` 垫片 → `system_preflight` 报 `wheeltec-robot.service is not active` 直接拆栈；
* 车端栈与 Gazebo 同时启动 → `sensor_preflight` 8 s 等不到 odom（改成延迟 12 s 起）；
* RViz 被 SIGINT 后刷 `failed to create guard condition`（拆栈后的噪声，不是根因）。

尚未做的对照（下一轮）：同一 `nav_verify` 里跑 `COST_INFL=0` 与 `=1` 各一组目标，
统计"计划最小净宽中位数 / 穿窄处比例"，把这套代价调参的影响量成数字。

---

## 10. L1/L2 卡死治理：5 轮 A/B 与**回退结论**（2026-10-26 收尾）

### 10.1 标定（离线，50 轮历史 timeline）

见 §9 与 `harness/analyze_failfast.py`。最终判据：滑窗 **20 s** / 位姿包围盒对角线 **≤0.20 m** /
净空证据（`front_min_range < 0.60 m` 或 `nav=blocked_*`）/ 非"在等"状态 → 连续 20 s 记一次命中；
同 2 m 区 60 s 内 **≥2 次**才动手。离线复算：命中 62 次、确认 19 次（0.38/轮）、误报 5/50 轮、覆盖 7/11 顶住类事件。
（原设想里的"里程 ≥0.6 m"被数据否掉：真卡死时里程 ≈0，会把事件全滤掉。）

### 10.2 在线 5 轮（禁用 Gazebo 无关，整栈同配置：PASS_W=1.05 / VISION_DECAY=20 / RECOVERY_BT=replan / FAILFAST=1）

| 轮 | 接触 | 卡死 | 最长原地挪动 | L1 命中 | L2 动作 | 禁入区拒绝 |
| --- | --- | --- | --- | --- | --- | --- |
| ffr1 | 0 | 2 | 5 s | 0 | 0 | 0 |
| ffr2 | 0 | 2 | 10 s | 0 | 0 | 0 |
| ffr3 | 0 | 2 | **82 s** | 4 | 2 | 0 |
| ffr4 | 0 | 2 | 5 s | 10 | 5 | 23 |
| ffr5 | 0 | 4 | **99 s** | 6 | 3 | 15 |

对照组 nosp（5 轮）：卡死 3.80/轮、卡住 2.20/轮、最长 13.4 s、接触 0、S0 5/5、覆盖 63.6 格。

### 10.3 判定：退化，回退

* 卡死/轮 3.80 → **2.40**（改善）；接触 0 → 0（S0 保持）；
* **最长原地挪动 13.4 s → 40.2 s（且出现 82 s / 99 s 两例）** —— 这正是用户最在意的那条，
  属于明确退化；
* 机制确实在工作（L1 命中 20 次、L2 动作 10 次、禁入区拒绝 38 次），但**没有缩短最长停滞**，
  说明"取消+换目标+拉黑"之后，新目标往往还是同一片墙角的邻近前沿（blacklist 半径 1.2 m
  只挡住了落点，没挡住"同一死角的另一侧"）；
* 覆盖格同时从 63.6 掉到 55 左右（探索量退化）。

按既定纪律（任何退化立即回退）：**`FAILFAST` 保持默认关闭**，代码与开关保留（可随时
`FAILFAST=1` 复现），回退动作只是"不启用"，无残留文件改动。

### 10.4 下一步建议（未做）

1. L2 的拉黑半径按"死角"而不是"落点"来划：用受阻区的自由连通域（`comp_r2`）当单位，
   否则总在同一个死角里换目标；
2. 与"受限倒车"解耦验证：最长停滞的根因是"车头贴死 + 不能退"，L1/L2 只能改`谁去`，
   改不了`怎么出`；
3. 该机制更适合放在 manager 侧（它手里有 no_progress 12 s 与 goal 生命周期），
   仲裁器侧只在"选谁"上做区域排除，避免两套计时器互相盖住。
