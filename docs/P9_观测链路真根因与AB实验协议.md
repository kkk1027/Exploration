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
