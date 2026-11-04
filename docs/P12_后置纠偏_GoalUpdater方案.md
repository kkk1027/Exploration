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

---

# 实施记录（2026-11-04）

## Step 1 完成（纯配置，已提交 dae3854 / 4fb5a01）
- `stemm_cartographer_nav_to_pose.xml`：`GoalUpdater` 包在 `ComputePathIfNeeded` Fallback **外层**
- `bt_navigator.goal_updater_topic = /goal_update`
- 60 s 短轮验证：BT 加载无错、8 次派发 / 2 次取消、接触 0，行为不变。

## Step 2 判定"无需改动"（有源码依据）
nav2 的 `NavigateToPose` 反馈 `distance_remaining` = **当前路径剩余长度**
（`nav2_bt_navigator/src/navigators/navigate_to_pose.cpp:147` `calculate_path_length`）；
manager 的 `_distance_to_active_goal()` 优先取该反馈（`_distance_source == 'feedback'`）。
⇒ 目标被原地改写后路径变化，看门狗的"剩余距离/净推进"自动跟随，**不需要额外订阅**。

## Step 3 完成（arbiter inplace，已提交 edab1e4 / 9074f62）
- 新参数：`veto_inplace`(默认 false) / `inplace_radius_m` / `inplace_tries` / `inplace_max`
  / `inplace_cooldown_sec` / `inplace_min_disp_m` / `inplace_min_width_m` / `inplace_min_robot_m`
  / `inplace_min_goal_m` / `inplace_fallback_cancel`；新话题 `/goal_update`、`/goal_correction`。
- 离线单测 `harness/test_veto.py`：10 组 10 断言全过（含"inplace 不调用 cancel"与"回落旧逻辑"）。

## 机制级验证（关键，PASS）
`harness/verify_goal_updater.py` 在真实轮次里注入：
```
[verify] 当前计划末点=9.12,-6.82  车=7.03,-5.30
[verify] 注入 /goal_update → 5.56,-7.32
[verify] 注入后 5 条计划；最接近注入点的末点=5.56,-7.32 距离=0.01 m
[verify] ✓ PASS：BT 已消费 /goal_update（重规划到注入点）
```
⇒ 订阅者 = `bt_navigator_navigate_to_pose_rclcpp_node`（BEST_EFFORT，与 RELIABLE 发布兼容）；
外层 `NavigateToPose` 全程未取消。**方案链路成立。**

## 踩到并修掉的两个真 bug
1. **参数被覆盖**：`nav_params.py` 每轮从 `patches/stemm_safe_nav2_params.pristine.yaml`
   **整份重写** YAML，Step1 写在运行副本里的 `goal_updater_topic` 被抹掉（实测计数 0）。
   ⇒ 已写入 pristine 并强制入库（该路径被 .gitignore 忽略，需 `git add -f`）。
2. **G\* 判据过严**：首版直接复用 migrate 的 `_veto_ok`（含 `veto_migrate_min_move=1.0 m`）
   与 `corridor_width=1.05 m`，导致 5 轮只纠偏 4 次。⇒ 解耦出 `_inplace_ok`：
   离车 ≥0.50 m、离原目标 ≥0.20 m、`cost_gate_reject` 通过、区域复杂度通过，
   净宽阈值单列 `inplace_min_width_m=0.70`，尝试次数 24→48。

## 第一轮 A/B（判据未达标，作为基线对照保留）
| 指标 | A=cancel(5 轮) | B=inplace(5 轮, 旧严判据) |
| --- | --- | --- |
| 取消次数 | 33 | 22 |
| 取消型迁移 | 24 | 13 |
| 原地纠偏 | 0 | **4（太少，判据过严）** |
| 无故打断 | 9 | 5 |
| 已知面积中位 | 148.6 m² | 148.2 m² |
| 卡死/停滞 | 22 / 202 s | 18 / 241 s |
| 接触 | 0 | **8（1 轮，该轮纠偏=0，与 inplace 无关）** |

> 结论：方向有效（取消 33→22、无故打断 9→5），但纠偏触发太少、且出现一轮既有形态的接触，
> 需要按 §"G* 判据放宽"重跑 B 组后再判定。

## Step 4 A/B 结果（2026-11-04，各组 5 轮 180 s，同口径）

| 指标 | A=现状(取消型) | B=inplace+兜底(ab4) | B'=inplace 不兜底(ab5) |
| --- | --- | --- | --- |
| 取消次数（合计） | 33 | **23（−30%）** | **0（但见下）** |
| 取消型迁移 | 24 | **10** | 0 |
| 原地纠偏 | 0 | **5** | 6 |
| 无故打断 | 9 | **5（−44%）** | —（轮次不健康） |
| 中途更换 | 29 | 22 | — |
| 已知面积中位 | 148.6 m² | 147.4 m²（−0.8%） | **79 m²（腰斩）** |
| 占据面积 | 7.8 m² | 6.9 m² | 3.4~3.8 m² |
| 卡死事件 | 22 | 26 | — |
| 停滞总秒 | 202 | **369（翻倍）** | 136~138 s/轮 |
| 物理接触 | 0 | **0** | 0 |

### 判读
1. **机制成立**（GoalUpdater 被 BT 消费、注入验证 0.01 m 命中），且 inplace 确实减少了取消
   （33→23，−30%）与无故打断（9→5，−44%），覆盖面积几乎不变（−0.8%），无接触。
2. **但纠偏触发率偏低（5 次/5 轮）**：后置判据的触发前提就是"落点邻域拥挤"，而我在
   3.5 m 半径内还要求"代价门 + BFS 可达（+净宽）"，实测多数时候找不到合格点 → 回落取消。
3. **"永不取消"（ab5）不可取**：取消降到 0，但停滞从 43 s/轮涨到 ~137 s/轮、覆盖腰斩到
   79 m²，另有两轮起栈失败 ⇒ 停车被换成了卡死。**必须保留 cancel 兜底。**
4. 因此**未达验收判据**（≤3 取消/5 轮、≤1 无故打断、停滞不升）。

### 下一步建议（待批准）
- **G\* 搜索质量**是唯一瓶颈：优先沿"当前计划回退"（rollback 档，最远 3.55 m）取点，
  再退回圆周采样；并把净宽判据改为"≥车宽+0.1"（0.57 m）而不是 0.70/1.05。
- 或者：把"落点不合格"的处置从"立刻改目标"改为"**先把该点拉黑，等本次目标自然结束后
  再改目标**"（延迟纠偏），彻底避免中途变向。
- 判据层面建议把"停滞总秒不升"作为硬约束优先于"取消次数"，避免再出现 ab5 那种
  "取消归零但车不动"的假达标。

## 取消归因（按坐标精确配对，5 轮/组）

| 取消来源 | A=现状 | B=inplace+兜底 |
| --- | --- | --- |
| **后置迁移**（P12 覆盖的那一类） | **21** | **9（−57%）** |
| watchdog 静止自愈 | 6 | 10 |
| 无匹配（静默） | 6 | 4 |
| **合计** | 33 | 23 |

**结论**：P12 把"后置纠偏"这一类取消从 21 降到 9（−57%，其中 5 次改为原地纠偏、不再停车）；
剩下的 14 次取消来自 **watchdog 静止自愈（10）与静默路径（4）**，不属于 P12 范围。
因此"整轮取消 ≤3/5 轮"这个判据**不是 P12 单独能达到的**，需要后续处理 watchdog 那条链路
（它本身是"车静止 → 取消目标换候选"的兜底，与"中途改目标"是两个问题）。
