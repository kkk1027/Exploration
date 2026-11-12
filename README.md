# Exploration — 全自主探索建图的仿真实验仓库

面向**实机自主探索功能开发**的**仿真探索链路实验仓库**：在 Gazebo 中用与车端一致的
Nav2 + Cartographer 栈复现实机探索行为，用于探索算法的 **sim-to-real** 验证、
回归测试与 A/B 实验。

本仓库**只做仿真侧实验**。车端（实机）代码在独立仓库中，不在此处维护。

---

## 这个仓库解决什么问题

实机自主探索建图有一个典型困境：**每次改动都必须上真车验证，成本高、周期长、不可
复现**——随机起点不同、房间光照不同、电池电量不同，同样的代码可能一次跑通一次撞墙。

本仓库的做法是：把实机探索链路完整搬到仿真里，让**同一个算法节点**在仿真中驱动
Gazebo 机器人与 Cartographer 建图，然后：

- 用**确定性启动 + 随机起点 + 固定时长**做可重复的单轮实验
- 用**七项自动验收 + 上百个离线分析脚本**做量化取证
- 用**配对 bootstrap** 判定改动是否真的有效（而不是"看着好像好一点"）

这样，实机上只做最后一步确认，绝大部分迭代在仿真里完成。

---

## 技术栈

| 项 | 版本 / 说明 |
|---|---|
| ROS | ROS 2 Humble |
| 仿真器 | Gazebo Classic 11 |
| SLAM | Cartographer 2D |
| 导航 | Nav2 1.1.20 |
| 场景 | `aws_small_house.world`（AWS RoboMaker 小户型，约 12 × 19 m） |
| 机器人 | 两轮差速，footprint 0.50 × 0.40 m，最大速度 ~0.5 m/s |

---

## 架构

### 核心链路

```
RRT 前沿检测 ──► /detected_frontiers
                       │
                       ▼
                前沿过滤/合并 ──► /filtered_goal_points
                                        │
                                        ▼
                            ┌───────────────────────┐
                            │   frontier_arbiter    │  ◄── /map, /odom_combined
                            │   （核心仲裁节点）      │  ◄── /plan, /global_costmap
                            │       ~4600 行        │  ◄── /vision/obstacle_grid
                            └───────────────────────┘
                                        │
                          /validated_frontier_goal
                                        │
                                        ▼
                                 frontier_merge ──► Nav2 NavigateToPose
                                                         │
                                                         ▼
                                                 Cartographer /map
```

`frontier_arbiter` 是整个系统的核心，负责：

- **可达性预筛**：BFS 连通性 + 沿路径实测最小净宽（沿法向双侧求和，不只取最近障碍）
- **候选闸门**：可脱困方向数 / 2 m 邻域占用率 / 死口袋判定
- **风险评分**：距离折现、转角惩罚、落点复杂度、沿路窄段与未知占比
- **候选排序**：当前主逻辑为「未知格数 × 距离折现」加权，可选「先远后近」两阶段模式
- **后置纠偏**：目标途中被判定不合格时，优先原地改向 / 沿路径中继 / 随机迁移，
  尽量避免直接取消
- **观测落盘**：每拍时间线、每次派发的候选特征、候选池快照

### 支撑节点

| 节点 | 作用 |
|---|---|
| `collision_probe` | 撞障取证（底盘/轮/雷达近距点），全程只读 |
| `escape_supervisor` | 脱困监督：卡住时接管，禁止非受控硬倒车 |
| `explore_watchdog` | 探索进度看门狗，无净进展时升级处理 |
| `frontier_merge` | 仲裁目标优先（带 TTL），到期回落 RRT 原始样本作兜底 |
| `vision_obstacle_grid` | 视觉障碍栅格，补雷达盲区（桌腿底座等） |
| `nav_plan_audit` | 规划审计，落盘规划失败原因 |
| `nav2_lifecycle_nudge` | Nav2 生命周期兜底 |
| `patch_sim_time` | 运行时给缺 `use_sim_time` 的车端节点补齐 |
| `session_driver` | 单轮会话驱动（到点结束建图、触发返航） |
| `wait_robot_ready` | 起栈就绪门控 |
| `fake_base` / `sim_shims` / `urdf_publisher` | 仿真适配垫片 |

---

## 目录结构

```
├── src/stemm_cartographer_exploration_sim/
│   ├── scripts/            # 全部节点（Python，无扩展名）
│   ├── launch/             # sim_explore.launch.py 为主入口
│   └── config/             # Nav2 / Cartographer 参数
├── harness/                # 实验与取证工具（100+ 脚本）
│   ├── round.sh            # ★ 单轮实验运行器（确定性启动 + 收尾 + 验收）
│   ├── evaluate.py         # 自动验收（11 项检查）
│   ├── metrics.py          # 指标提取
│   ├── test_*.py           # 离线回归测试
│   └── analyze_*.py        # 各类离线归因分析
├── patches/                # 对车端仓库的补丁与 pristine 基线
├── docs/                   # 各阶段实施方案与实验报告
├── logs/                   # 每轮实验产物（体积大，已在 .gitignore 中排除）
├── runs/                   # 每轮摘要 JSONL（历史上已入库的保留，新增的默认忽略）
└── shots/                  # 关键证据截图
```

---

## 快速开始

### 环境准备

本仓库依赖**多个叠加工作空间**，需按顺序 source（后者覆盖前者）：

```bash
# 1. 基础仿真环境（域隔离 + Gazebo + 通用依赖）
source $HOME/stemm_sim_ws/domain_guard.sh      # 设定 ROS_DOMAIN_ID 隔离
source /opt/ros/humble/setup.bash
source $HOME/stemm_sim_ws/install/setup.bash

# 2. 探索专用依赖
source $HOME/stemm_explore_ws/install/setup.bash

# 3. 本仓库
source $HOME/stemm_exploration_sim_ws/install/setup.bash

# 4. 车端仓库（可选，用于覆盖特定包，如修补后的 nav2_behavior_tree）
[ -f $HOME/wheeltec_ros2/install/setup.bash ] && \
    source $HOME/wheeltec_ros2/install/setup.bash
```

> 路径按你的实际部署调整。`harness/round.sh` 顶部已封装上述顺序。

### 编译

```bash
cd <本仓库根目录>
colcon build --symlink-install
```

Python 节点走 `--symlink-install`，改 `.py` 后**无需重新编译**即可生效。

### 跑一轮

```bash
./harness/round.sh <标签> <探索秒数>
# 例：
./harness/round.sh baseline 180
```

单轮约 **8 分钟**（180 s 探索 + 起栈 + 返航 + 收尾）。产物：
- `logs/round_<时间戳>_<标签>/report.md` —— 体检报告
- `logs/round_<时间戳>_<标签>/launch.log` —— 全量日志
- `logs/round_<时间戳>_<标签>/cand_pool.jsonl` —— 候选池快照
- `runs/<时间戳>-<标签>.jsonl` —— 单轮摘要

### 跑 A/B

```bash
for i in $(seq 1 10); do
  VAR_A=0 ./harness/round.sh myexp_a 180
done
for i in $(seq 1 10); do
  VAR_A=1 ./harness/round.sh myexp_b 180
done

python3 harness/p4_ab.py myexp_a myexp_b                  # 方向与机制归因
python3 harness/p13_stats.py 'logs/round_*myexp_a*' 'logs/round_*myexp_b*'   # 配对 bootstrap
```

> **10 轮/组是下限**。本项目的实测经验：单轮方差极大，5 轮以内经常得出相反结论。

---

## 验收标准

每轮由 `harness/evaluate.py` 自动判定，当前共 **11 项检查**：

| 项 | 判据 |
|---|---|
| `preflight` | 起栈前置检查通过 |
| `tf_healthy` | TF 链路健康，无超时 |
| `nav2_active` | Nav2 四节点全部 ACTIVE |
| `frontier` | 确实发生了前沿探索（有有效派发） |
| `mapping_done` | 到点正硬触发 `set_mapping_done` |
| `return_home` | 返航完成（以车端 `nav_state` 完成状态为准） |
| `map_saved` | 地图成功存盘 |
| `no_contact` | 物理接触为 0（底盘/轮） |
| `no_reverse` | 非受控硬倒车为 0 次 |
| `vision_cells` | 视觉障碍栅格正常输出 |
| `vision_layer_ok` | 视觉图层已正确注入代价地图插件链 |

> 前 7 项是最初的验收集（见 `EXPLORATION_SIM_验收记录.md`），
> 后 4 项是随撞障治理与视觉层接入逐步追加的。

另：每轮收尾时要求 `gzserver=0` 且无进程残留。

---

## 实验记录（docs/）

按阶段归档，每份包含**问题现象 → 根因取证 → 改动方案 → A/B 结果 → 残留下一个问题**：

| 文档 | 主题 |
|---|---|
| `EXPLORATION_SIM_验收记录.md` | 仿真接入的稳定性验收 |
| `P0_脱困与撞障治理.md` | 卡死与撞障 |
| `P1_候选生成缺陷清单.md` | 候选生成链路 |
| `P5_候选生成优化方案.md` | 候选生成重构 |
| `P6_V1候选闸门验证报告.md` | 候选硬门（含一次回滚） |
| `P8_观测工作流v2与P0窄处否决.md` | 观测链路 |
| `P9_观测链路真根因与AB实验协议.md` | AB 实验协议 + 真根因 |
| `P10_地图行序镜像_根因与归档.md` | 地图镜像根因 |
| `P12_后置纠偏_GoalUpdater方案.md` | 后置纠偏 |
| `P13_实施说明.md` | 宽度测量修正 / 窄道硬门拆分 / 原地改向 |
| `P14_候选落点倾向.md` | 先远后近两阶段排序 |

`docs/WORKFLOW.md` 记录了完整的实验工作流与观测字段口径。

---

## 重要约定

### 1. 只排序，不改生成

候选**生成**（RRT 采样、前沿提取、过滤）与候选**排序/仲裁**严格分层。
排序层可以任意重排、降级、否决，但**不允许回头改生成策略**——否则无法归因到底是
采样变好还是排序变好。

### 2. 新功能一律默认关闭

所有新增打分项、阶段机、准入判据，默认值必须等价于改造前行为。
A/B 时用环境变量打开，这样基准臂与改造臂跑的是**同一份代码**，避免"两个版本差异"污染。

### 3. 退化先回滚

任何改动若在 10 轮 A/B 中造成停滞/覆盖/接触任一项退化，先回滚再分析，
不在"半好半坏"状态下继续叠加改动。

### 4. 探针优先

改打分逻辑前先补探针（候选池快照、派发特征、判决记录）。
没有归因数据的改动，退化时只能看到"停滞变长了"，无法定位。

---

## 已知限制

- **单一世界**：全部实验在 `aws_small_house.world` 上完成，异世界泛化未验证。
- **返航判定偶发 FAIL**：基线批次亦如此（11/11 FAIL 为历史常态），探索段指标不受影响，
  但全绿验收需单独看 `return_home` 项。
- **部分车端路径硬编码**：`harness/` 与 `patches/` 中有绝对路径引用，移植时需调整。
- **`patches/` 中的 pristine 文件**是对车端仓库打补丁的基线，不是本仓库的源码备份。

---

## 相关仓库

- **车端（实机）仓库**：Nav2 / Cartographer / 传感器驱动 / 底盘控制
- 本仓库通过 `patches/` 对车端做最小侵入式修改，并通过最后 `source` 车端 overlay
  让修补后的包覆盖系统版本

---

## 许可

本仓库为内部开发实验仓库，未公开发布许可。
引用的第三方组件（Nav2、Cartographer、Gazebo、AWS RoboMaker 场景等）遵循其各自许可。
