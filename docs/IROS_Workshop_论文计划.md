# IROS Workshop 论文编写计划

*创建：2026-11-10*
*目标会议：IROS Workshop（具体 CFP 待定，格式按 workshop 通常的 4~6 页 + 参考文献）*
*工作标题（暂定）：**Typed Frontier Output: Propagating Feasibility Verdicts Across
the Exploration–Navigation Boundary***

---

## 0. 一句话定位

我们不提出「更好的前沿生成器」，而是提出**前沿生成器向导航层传递可行性判决的接口契约**，
并用一组负面结果证明：**在 Gazebo + Nav2 + RRT 这条链路上，只改生成侧的评分/门限
对派发质量无显著作用，瓶颈在跨层信息丢失**。

---

## 1. 为什么是这个题目（可写进 Introduction 的立论）

### 1.1 三轮 A/B 的负面结果（本文最强动机）

在同一环境、同一机器人、同一探索时长的交错 A/B 下（每组 5 轮，180 s）：

| 实验 | 改动 | 关键指标变化 |
|---|---|---|
| `p13ab` | 6 项结构化风险评分 + 脱困两轮搜索 | 距离 5.05→5.14 m（**变差**），覆盖 40→72.5% |
| `p13c` | 修 P13-2A 的 raw 分支硬门漏洞 | 距离 4.73→5.26 m（**变差**），转角 70→83°（**变差**） |
| `p13d` | 三处硬门 → 软约束（供给侧放开 3 个数量级） | `route_soft_pass` 0→991/轮，`published` 32.6→**32.4**（**无变化**） |

`p13d` 那一行是本文的核心证据：**供给侧放宽三个数量级，派发量不动。**

### 1.2 根因：判决从未离开生成器

`p13d` B 组 5 轮均值：

| 统计 | 值 | 含义 |
|---|---|---|
| `raw_seen` | 165.6 /轮 | 生成器产出的原始目标 |
| `raw_deadend` | 189.8 /轮 | 其中被判"无逃生方向"（死口袋） |
| `raw_publish` | **1.6 /轮** | 判决后真正放行 |
| `merge_raw` | 35~93 /轮 | **下游仍转发的未评分目标** |
| 派发时带评分特征的比例 | **16%** | 84% 的派发无评分数据 |

⇒ 生成器判死了 98.6%，下游却转发了 35~93 条。**判决被绕过。**

### 1.3 为什么现有工作没解决

（依据 `docs/P13_文献调研.md` 的逐仓库核验）

| 工作 | 有无跨层判决传递 |
|---|---|
| `hasauino/rrt_exploration`（IROS'17） | ❌ 只有 `hysteresis_radius` 层内滞后 |
| `m-explore-ros2` | ❌ 只有 PR #51 的层内 blacklist |
| `SMMR-Explore`（ICRA'21） | ❌ RRT 仅作对照基线 |
| `TARE`（RSS'21 最佳论文） | ❌ 自带 planner，无跨层概念 |
| `FUEL`（RA-L'21） | ❌ `.pcd` 仿真，无 Nav2 |
| `GBPlanner`（IROS'19） | ❌ 自有 `pci_mav` |

**六个工作，零个有跨层判决传递。** 这不是巧合：UAV 研究不存在地面楔死问题，
而地面工作的探索层与导航层通常由同一课题组一并实现，不需要接口契约。

---

## 2. 贡献声明（三条，按强度排序）

**C1（主）判决传递接口**
定义 `feasible / dead_end / too_narrow / unreachable / over_dist` 五类判决，
以带 TTL 的轻量消息在探索层与导航层间传递；下游对负判决目标拒绝兜底转发。
提出**饿死保护**（`force_after_sec`）作为判决强度的对偶约束。

**C2（指标）Verdict Bypass Rate（VBR）**
$$\text{VBR} = \frac{N_{\text{blocked}}}{N_{\text{blocked}} + N_{\text{forwarded}}}$$
度量「被上游否决、若无契约本会被下游派发」的比例。VBR=0 表示契约形同不存在；
VBR=1 表示全靠拦截（下游会饿死）。**该指标不依赖具体算法，可跨实现比较。**

**C3（systems note）Nav2 行为树的跨目标状态污染**
Nav2 `GoalUpdater` 节点的 `last_goal_received_` 是实例成员且不随 `NavigateToPose`
目标重置，导致一次 inplace 更新污染所有后续目标：`ComputePathToPose` 规划到机器人
自身位置，`path.poses.back()` ≈ 机器人位姿，于是**收到目标后 30 ms 就报"到达"**。
我们实测基线中 **79.2%** 的"到达"属此类（对 5.456 m 的目标，隐含速度 182 m/s）；
移除该节点后 10/10 轮归零，`reached` 位置误差从 2.5~7.0 m 收敛到 0.117~0.126 m。

> C3 是意外发现但可独立成立，建议以 **systems note / extended abstract** 形式
> 单独投，或作为本文 §5 的一小节。它对我们自己的价值已经兑现（幻影率归零）。

---

## 3. 实验设计（这是评审最会挑的部分）

### 3.1 必须补的实验（按优先级）

| # | 实验 | 目的 | 轮次 | 预计机时 |
|---|---|---|---|---|
| E1 | `VERDICT=0` vs `VERDICT=1`（单变量） | 主结果：VBR 与派发质量 | 10+10 | 约 3 h |
| E2 | `force_after_sec` 扫描 {8, 15, 30} | 判决强度 vs 饿死的 Pareto 曲线 | 3×5 | 约 2.2 h |
| E3 | 2 个 world（`aws_small_house` + 一个开阔场景） | 泛化性，回应"狭窄环境会不会饿死" | 2×10 | 约 3 h |
| E4 | `explore_lite` 基线（同 world/URDF/时长） | 回答"距离卡 5 m 是不是 RRT 的锅" | 5 | 约 45 min |
| E5 | 消融：去掉饿死保护 | 证明它不是装饰 | 5 | 约 45 min |

**合计约 10 h 机时。** 按每轮 8.8 min 算，含间隔约 12 h 墙钟。

### 3.2 主指标（全部已有工具链支持）

| 指标 | 工具 | 现状 |
|---|---|---|
| 取消率 / 派发距离 / 转角 | `harness/analyze_dispatch_risk.py` | ✅ 已有 |
| 幻影率（物理不可能到达） | 同上 | ✅ 已有 |
| VBR / 拦截 / 饿死放行 | 同上（本次新增） | ✅ 本次已接 |
| 覆盖率 | `harness/p13_ab_report.py` | ✅ 已有 |
| 探索总时长 / 路径长度 | 需从 `timeline.jsonl` 聚合 | ⚠️ 需补脚本（小） |

### 3.3 统计处理（必须做，否则 5 轮方差会被攻击）

`p13d` 的逐轮覆盖率：A 组 41.5/46.8/40.2/33.3/26.2，B 组 50.0/40.5/19.8/32.6/47.0。
**组内极差达 30 pt**，5 轮完全不足以支撑结论。

- 每条件 ≥10 轮
- 报告中位数 + 95% bootstrap CI（1000 次重采样）
- 配对比较：交错执行（a1 b1 a2 b2…）已天然配对，用配对 bootstrap
- **不要报均值 ± 标准差**（分布明显非正态，且有小样本离群）

---

## 4. 论文结构（4~6 页）

| 节 | 页 | 内容 | 数据状态 |
|---|---|---|---|
| Abstract | 0.2 | 「供给侧放开三个数量级，派发量不动」+ 判决传递 | ✅ |
| §1 Intro | 0.8 | Fig.1：判决产生 → 被绕过 → 派发 → 楔死/取消 的链路图 | 需画 |
| §2 Related | 0.7 | RRT 前沿（Umari'17）、FIS（FUEL）、blacklist（PR#51）、TARE；点明跨层契约空白 | ✅ 已调研 |
| §3 Problem | 0.8 | 形式化：生成器知道 $v(g)$，输出却是裸 $g$；下游效用 $U(g)$ 缺项 | 需写 |
| §4 Method | 1.0 | 判决定义、TTL、饿死保护、VBR | ✅ 已实现 |
| §5 Results | 1.5 | E1~E5 表格 + Pareto 图 | ⚠️ 待跑 |
| §6 Discussion | 0.5 | 局限：单机器人、2D、需 Nav2 兼容层 | — |
| §7 Conclusion | 0.2 | — | — |

### §3 的形式化（简要）

设生成器对每个候选前沿 $g$ 已计算判决 $v(g) \in \mathcal{V}$，
但当前接口只输出 $g$。下游以效用 $U(g)$ 排序派发：

$$U(g) = \underbrace{u(g)\cdot p(d(g))}_{\text{信息收益折现}} - \underbrace{r(g)}_{\text{路径风险}}$$

**若 $v(g)$ = `dead_end` 而 $U(g)$ 仍排名靠前，则 $g$ 必被派发。**
这正是我们观测到的：`raw_deadend` 189.8/轮 而 `raw_publish` 仅 1.6/轮 ——
生成器清楚 $v(g)$，但 $U(g)$ 里没有这一项，下游无从知晓。

⇒ 契约的作用是让 $U(g) \to -\infty$ 当 $v(g) \in \mathcal{V}^-$，除非触发饿死保护。

---

## 5. 风险与对策

| 风险 | 概率 | 对策 |
|---|---|---|
| **E1 无效**（判决传递不改善派发质量） | 中 | 这本身也是有效结果："判决传递只降低 VBR，不改善质量 ⇒ 瓶颈在判决本身的质量" → 转向研究判决判据（`raw_min_esc_dirs` 等） |
| 评审质疑"这只是修 bug" | 高 | 强调 VBR 是**可迁移指标**，且饿死保护把它变成**设计空间**而非一次性修补；E2 的 Pareto 曲线是关键 |
| 单环境泛化 | 高 | E3 必须做，且要含一个开阔场景 |
| 与 `m-explore-ros2` 关系不清 | 中 | Related 里主动说明：本方法**不依赖 RRT**，是接口层的；blacklist 是层内，本工作是跨层 |
| C3 被指"已知问题" | 中 | 检索确认是否有已发表的记录；若无则是新颖的 systems finding。**需在投稿前查一遍 nav2 issue tracker** |
| 真机缺失 | 中 | 若时间允许补一段真机视频（可选）；否则在 Limitation 里明说 |

---

## 6. 时间线（建议）

| 阶段 | 内容 | 依赖 |
|---|---|---|
| **W1（本周）** | 代码已就绪 ✅；补 E2/E5 的 sweep 脚本；补探索时长聚合脚本 | — |
| **W2** | 跑 E1（10+10，约 3 h 机时）+ E4（explore_lite 基线） | 机器空闲 |
| **W3** | 跑 E3（2 world × 10）；画 Fig.1 与 Pareto 图 | W2 结果 |
| **W4** | 写 §1–§4；跑统计（bootstrap CI） | W3 数据 |
| **W5** | 写 §5–§7；内部评审；查 nav2 issue tracker 确认 C3 新颖性 | — |
| **W6** | 定稿、格式对齐目标 workshop CFP | 需先确定投哪个 workshop |

**关键路径是 E1/E3 的机时**，不是写作。建议尽早开跑。

---

## 7. 与现有资产的对应关系

| 论文元素 | 已就绪的代码/数据 | 位置 |
|---|---|---|
| 判决传递实现 | `verdict_enabled` / `verdict_ttl` / `verdict_radius` / `force_after_sec` | `frontier_arbiter`、`frontier_merge` |
| VBR 指标 | `_verdict_report()` | `harness/analyze_dispatch_risk.py` |
| 不变量测试 | 38 项断言 | `harness/test_verdict_propagation.py` |
| 单变量 A/B | `VERDICT=0/1` 分组 | `harness/p13_ab.sh` |
| 负面结果数据 | `p13ab` / `p13c` / `p13d` 共 30 轮 | `logs/round_*` |
| 文献支撑 | 7 个仓库的逐项核验 | `docs/P13_文献调研.md` |
| 幻影 bug 记录 | 根因 + 复现条件 | `docs/P13_实施说明.md` §8 |

---

## 8. 待办清单（勾选式）

- [x] 判决传递实现（arbiter 发布 + merge 消费）
- [x] 饿死保护
- [x] VBR 指标
- [x] 离线不变量测试（38 项）
- [x] 单变量开关接入 A/B 脚本
- [ ] E1：`VERDICT=0/1` 各 10 轮
- [ ] E2：`force_after_sec` 三档扫描
- [ ] E3：第二个 world（开阔场景）
- [ ] E4：`explore_lite` 同环境基线
- [ ] E5：去掉饿死保护的消融
- [ ] 探索时长 / 路径长度聚合脚本
- [ ] bootstrap CI 统计脚本
- [ ] Fig.1 链路图
- [ ] Pareto 曲线图
- [ ] 确认目标 workshop CFP 与页数限制
- [ ] 查 nav2 issue tracker 确认 C3 新颖性
