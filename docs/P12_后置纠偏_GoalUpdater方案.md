# P12 · 后置纠偏改造方案：把"取消目标"换成"原地改目标"（nav2 GoalUpdater）

> 目标：保留**后置**判据（它能看见局部代价地图之外的不确定候选），但把动作从
> 「取消当前目标 + 重新派发」改成「**在不取消 NavigateToPose 动作的前提下，把目标迁移到最近的合格自由点**」。
> 状态：方案与可行性已核实，**未实施**，等批准。

## 1. 现状（已实测锁定）

`frontier_arbiter` 的后置判据触发后走 `veto_migrate`：**先 `_do_veto_cancel()`（按 UUID 取消）**，
再发布迁移后的新目标（frontier_arbiter:1509-1522）。5 轮实测：
- nav2 目标被取消 30 次，其中 **24 次（80%）来自这条迁移路径**；
- 触发判据：`occ_frac_r2 ≥ 0.30` 或 `esc_dirs < 3`；额度 6 次/轮（r2/r4 打满）；
- 迁移发生在已跑 8~14 s、剩余 5 m 时 → 车停车重规划，是最直接的"走一半停"来源。

## 2. nav2 原生机制：`GoalUpdater`（无需编译、装机版自带）

- 插件：`/opt/ros/humble/lib/libnav2_goal_updater_node_bt_node.so`，导出 BT 节点 `GoalUpdater`
  （端口 `input_goal` / `output_goal`，参数 `goal_updater_topic`，默认 `goal_update`）。
- 语义（`goal_updater_node.cpp`）：
  ```cpp
  getInput("input_goal", goal);
  callback_group_executor_.spin_some();
  if (last_goal_received_.stamp > goal.stamp) goal = last_goal_received_;  // 只接受更新的 stamp
  setOutput("output_goal", goal);
  return child_node_->executeTick();     // 子节点用新目标重跑
  ```
  ⇒ **改写正在运行的 BT 的目标，外层 `NavigateToPose` 动作不取消**。

### 2.1 为什么"到达 G\* 就算成功" —— 成立（源码为证）
`nav2_controller/src/controller_server.cpp`：
```cpp
439:  end_pose_ = path.poses.back();          // 目标校验用的是"当前路径末点"
590:  return goal_checkers_[...]->isGoalReached(pose.pose, transformed_end_pose.pose, ...);
```
新路径的末点就是 G\*，车到 G\* 即 `Reached the goal!` → 动作 **SUCCEEDED**（不是 abort、不是 cancel）。

### 2.2 为什么"改了目标就会重规划" —— 成立（本仓库 BT 已经具备条件）
当前 BT（`stemm_cartographer_nav_to_pose.xml`）：
```xml
<Fallback name="ComputePathIfNeeded">
  <ReactiveSequence name="UseCurrentPathIfValid">
    <Inverter><GlobalUpdatedGoal/></Inverter>     <!-- 目标没变 → 允许复用旧路径 -->
    <IsPathValid path="{path}"/>
  </ReactiveSequence>
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
</Fallback>
```
`globally_updated_goal_condition.cpp`：`{goal}` 与上次一致 → FAILURE，不一致 → SUCCESS。
所以只要 `{goal}` 被改写，`Inverter` 立刻失败、`UseCurrentPathIfValid` 失败，
Fallback 落到 `ComputePathToPose` 用 **G\*** 重规划。

⚠️ 关键放置细节：`GoalUpdater` 必须包在 **Fallback 外层**（不是包 `ComputePathToPose`），
否则同一 tick 内 `GlobalUpdatedGoal` 先跑、还看不到新目标，会一直复用旧路径（死锁）。

## 3. 方案设计

### 3.1 数据流
```
arbiter 后置判据命中（判据不变）
  ├─ 计算 G* = 离原目标最近的"合格自由点"（见 3.2）
  ├─ 发布 geometry_msgs/PoseStamped → /goal_update（frame=map, stamp=now）
  │     └─ bt_navigator 的 GoalUpdater 改写 {goal} → 强制重规划到 G*（动作继续）
  └─ 同时发布 /goal_correction（含 reason/原目标/G*/可行性余量）供 manager 与统计使用
```

### 3.2 G\* 的搜索（确定性，复用现有原语）
1. 以原目标为圆心、半径 `R`（默认 2.0 m），0.05 m 网格枚举候选；
2. 逐点过滤（全部复用 `frontier_arbiter` 现成判据）：
   - `cost_gate_reject(x,y)` 通过：落点代价 < `cost_gate_hard(99)`；0.35 m 盘内无致命；
     盘内已知占据占比 ≤ `cost_gate_free_ratio(0.80)`；
   - 全局 costmap 上**从车当前位置 BFS 可达**（只走非致命格）；
   - 运动学/净宽：复用 `path_and_width`（BFS + 沿途最小净宽）要求净宽 ≥ `PASS_W`，或至少 ≥ 车宽+0.1；
3. 取"到原目标欧氏距离最近"的合格点作为 G\*；
4. R 内无合格点 ⇒ **不纠偏**，只记黑名单（回落到现状：目标继续跑完）。

### 3.3 必须同步改的一处（否则前功尽弃）
manager 的 `no-progress` 看门狗是按**它自己派发的目标**算剩余距离的；
`/goal_update` 只改 BT 内部目标，manager 不知道 ⇒ 会判"没有进展"→ 走它自己的取消路径。
⇒ manager 需订阅 `/goal_update`（或 `/goal_correction`）并更新 `progress` 用的目标点。约 10 行。

### 3.4 频率与安全边界
- 同一目标最多纠偏 3 次（新参数 `correction_max`）；两次纠偏间隔 ≥3 s（沿用 `veto_cooldown`）；
- G\* 与上次纠偏点距离 <0.3 m 则不重复发布（避免 BT 反复重规划、规划器 churn）；
- **只在探索阶段纠偏**（`_explore_phase()`）；返航/手动目标**不纠偏**（返航要求精确到原点）；
- 保留 `veto_migrate_mode = cancel | inplace`，默认先 `cancel`（灰度），验证后切 `inplace`。

## 4. 可行性分析

| 项 | 结论 | 依据 |
| --- | --- | --- |
| 不取消动作改目标 | 可行 | `GoalUpdater` 装机自带，只改 BT 黑板 goal |
| 到 G\* 判成功 | 可行 | `controller_server.cpp:439` 用 `path.poses.back()` |
| 改目标即重规划 | 可行 | `GlobalUpdatedGoal` + 现有 BT 的 `Inverter` 结构 |
| 规划器支持（SMAC Hybrid） | 可行 | SMAC 不校验目标代价，会对 G\* 正常出路径 |
| G\* 存在性 | **需实测** | 24 次触发点的 R=2 m 内是否都有合格点：用 `cost_gate_reject` 离线回放统计 |
| 与返航的隔离 | 可行 | 按阶段门控（`_explore_phase`） |
| 编译/部署成本 | 极低 | 仅 BT XML 一行 + 两个参数 + arbiter/manager 各一处小改 |
| 回退 | 一行/一个参数 | `veto_migrate_mode=cancel` 即回到现状；XML 改动可 git revert |

### 风险与缓解
| 风险 | 缓解 |
| --- | --- |
| manager 看门狗误判（3.3） | 必须同步；先用 1 轮冒烟验 `no-progress` 不再误触发 |
| G\* 仍在拥挤区 → 反复纠偏 | 上限 3 次 + 间隔 3 s + 位移阈值；超限回落 cancel |
| 到点精度下降（停在 G\* 而非原目标） | 只在探索阶段；返航不纠偏；统计里记录纠偏距离 |
| 纠偏点不可达（BFS 通过但控制跟不动） | 复用 `path_and_width` 净宽判据 + 纠偏后 3 s 内不重复纠偏 |
| BT 插件未加载 | `bt_navigator` 参数 `plugin_lib_names` 需含 `nav2_goal_updater_node_bt_node`（需核实本机参数表） |

## 5. 验收（A/B，各 5 轮，同一协议）

| 指标 | 现状 | 期望 |
| --- | --- | --- |
| 中途取消（nav2 goal canceled） | 30 / 5 轮 | **≤3 / 5 轮** |
| 无故打断（analyze_interruption） | 10 / 5 轮 | **≤1 / 5 轮** |
| 纠偏次数（新指标） | — | 与取消次数同量级，且**不产生停车** |
| 覆盖面积中位 | 153.5 m² | 不低于 −5% |
| 卡死/停滞中位 | 3 次 / 36 s | 不上升 |
| S0 物理接触 | 0/0/0/0/0 | 0 |

## 6. 实施顺序（每步独立提交、可回退）
1. `bt_navigator` 参数 + BT XML 加 `GoalUpdater`（纯配置，先 `veto_migrate_mode=cancel` 不变行为）；
2. manager 订阅 `/goal_update` 同步跟踪目标 + 离线自测；
3. arbiter 增加 `inplace` 模式（G\* 搜索 + 发布 `/goal_update` + `/goal_correction`）；
4. 5 轮 A/B（`cancel` vs `inplace`），按 §5 判据决定是否切换默认。
