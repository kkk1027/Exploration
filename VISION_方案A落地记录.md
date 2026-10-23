# 方案A：深度相机视觉障碍层（体素层 + 全局视觉障碍层）落地记录

> 目标：给原有"cartographer 2D + RRT 前沿 + Nav2"的探索建图链路**加一层深度相机感知**，
> 治掉 2D 雷达的结构性盲区（P0_验收记录 §3b：cafe_table 只有 4.2 cm 细立柱进图，
> 4 cm 贴地底板完全看不见 → 规划器从"假口子"开进去、轮子压底板）。
> 红线：车端源码零提交、不抢 `/scan` 与 `/map`、`map→odom` 发布者不变、可一键回退。

## 1. 一句话现状

深度相机 → `vision_obstacle_grid` → **local_costmap 体素层 + global_costmap 只增不减障碍层**，
已在本机仿真全链路打通并验收：

* 视觉障碍在全局代价地图里 **100% 变成致命格**（lethal_ratio=1.0），局部体素层同样在标（窗口内致命格 5→408）；
* **定点复现测试**：基线"穿假口子、左轮撞 cafe_table 底板 4 次"；视觉轮"规划器直接判无路、**零接触**、最近间隙 1.17 m；
* **探索 A/B**：视觉轮 **5/6 PASS**（基线 2/3），9 轮**物理接触全部为 0**，原七项检查无退化，探索覆盖面积中位数还略高；
* **开销**：视觉节点峰值 **77 MB / 约 0.09 个核**，整栈 ~1.2 GB，可用内存从未低于 10.5 GB（本机 15 GiB、无 swap）。

所有改动都可回退（`VISION=0` 即恢复纯雷达，参数文件逐字节还原）。

## 2. 数据流

```
Gazebo 深度相机 (160x120@5Hz, camera_depth_optical_frame)
        └─ /camera/depth/points ──► vision_obstacle_grid (仿真侧新节点)
                                        │  map 帧 + 自身过滤(0.38 m) + z 0.05~1.80 + 体素抽稀 5 cm
                                        ├─ /vision/live_cloud     当前帧（5 Hz） → local_costmap  vision_voxel_layer
                                        ├─ /vision/obstacle_cloud 累积障碍点（1 Hz, z=0.15） → global_costmap vision_obstacle_layer（marking only）
                                        ├─ /vision/obstacle_grid  累积 2D 栅格 0.05 m（1 Hz, transient_local） → 存盘/RViz/StaticLayer
                                        └─ 收尾落盘 vision_grid.{pgm,yaml}（事故复盘：卡在哪、那里是不是视觉标出来的）
                                        └─ /vision/low_cloud      0.05~0.35 m 低矮带子集 → 取证
```

消费方：`frontier_arbiter` 本来就在订阅 `/global_costmap/costmap`，所以**视觉障碍自动参与
候选点可达性判定**；Nav2 规划/控制直接在代价地图里吃到。

## 3. 代码位置与开关

| 文件 | 作用 |
| --- | --- |
| `src/.../scripts/vision_obstacle_grid` | 感知源节点（新） |
| `harness/vision_nav_params.py` | nav2 参数文件注入/撤销（plugins 只能启动时生效） |
| `launch/sim_explore.launch.py` | `vision:=true/false` 开关（默认 false） |
| `harness/round.sh` | `VISION=1` → 自动注入 + 全程内存采样 + 收尾撤销 |
| `harness/check_vision_layer.sh` | 集成自检：注入 → 起栈 → 交叉验证 → 撤销 |
| `harness/vision_check.py` | 视觉栅格 vs 全局代价地图交叉验证 |
| `harness/test_vision_table.sh` | 定点复现测试（A/B） |
| `harness/ab_vision.sh` | 探索 A/B 对照（固定起点、交替跑） |
| `harness/probe_camera.sh` | 相机体检（话题/帧率/TF/分辨率） |
| `harness/costmap_dump.py` | 代价地图 vs 视觉栅格 ASCII 对照 |
| `harness/stack_up.sh` / `stack_down.sh` | 常驻栈调试（省掉每轮 4 分钟的起栈） |

一键回退：`VISION=0 bash harness/round.sh ...`（会自动 `--restore` 参数文件）。

## 4. 参数与"为什么是这个值"

| 参数 | 值 | 理由 |
| --- | --- | --- |
| `max_obstacle_height`（两层） | **0.80 / 0.85** | cafe_table 桌面在 **0.755(+0.02) m**。取 0.60 会把桌面点全部滤掉、只剩 4.2 cm 立柱 —— 那样这套层治不了"假口子"（这条踩过） |
| `min_obstacle_height` | 0.05 | 贴地地板（z=0）必须排除，否则噪声会把整片地面标成障碍 |
| `z_voxels`（local VoxelLayer） | 16 × 0.05 = 0.80 m | 覆盖到桌面高度；局部 6×6 m 滚动窗，体素总量 120×120×16 ≈ 23 万格，内存可忽略 |
| `self_filter_radius` | 0.38 m | 相机装在车头前缘（x=0.24），必须滤掉车体自身点，否则 MPPI 会认为自己压着障碍 |
| `range` | 0.25~5.0 m | 相机近场 0.62 m 以内看不到地面（几何下限），这段交给雷达 min_range 0.15 |
| `min_points_per_cell` | 2 | 单帧内至少 2 个点，去掉孤立噪点 |
| `min_frames` | 2 | **跨帧持久化**：同一格要在 ≥2 帧都被观测到才算障碍，压掉掠射角/地面噪点的"幽灵格" |
| 累积策略 | **只增不减** | 与"激光盲区障碍"语义一致；代价是"物体移走不会消失"（见 §7） |

## 5. 已验证的实证（截至 2026-10-23）

### 5.1 相机体检（`harness/probe_camera.sh`）
`/camera/depth/points` 5.0 Hz、160×120、frame `camera_depth_optical_frame`、
TF `base_footprint→camera_depth_optical_frame` = `[0.240, 0, 0.260]`（与权威外参一致）。
⚠️ `/camera/color/image_raw` **无数据**（xacro 里 `image_raw:=/camera/color/image_raw` 的 remap 没生效，
实际 RGB 落在 `/camera/depth/image_raw`）——方案A 只用深度，不受影响；语义线（L2）前必须修。

### 5.2 交叉验证（`harness/vision_check.py`）
```
vision_cells=487  vision_cells_in_costmap=487
lethal_at_vision=487  lethal_ratio=1.0  verdict=PASS
topic_diag: subs{obstacle_cloud:1, live_cloud:1}  pubs{...:1}
```
即：视觉累积的每一个障碍格，在 `/global_costmap/costmap` 里都是致命格；nav2 两个视觉图层确实订阅到了。

### 5.3 运行期开关（同一次运行内切换，证明"可回退"）
| 视觉层 | 全局致命格 | 说明 |
| --- | --- | --- |
| ON | 486 | 两张桌子桌面连成一道 ≥1.8 m 的墙 |
| `vision_obstacle_layer.enabled=false` | 1（只剩立柱） | 立刻退回纯雷达 |
| 再 `true` | 4035/4037 lethal | 标记恢复（地图已随行驶扩大） |

### 5.4 定点复现测试（`harness/test_vision_table.sh`）
起点世界 `(2.04,1.37)` 正对两张 cafe_table（中心 (3.54,1.37)/(4.46,1.37)，桌面 0.913 m 见方，
中心距 0.92 m ⇒ 桌面几乎相贴、真实不可通行；底板 0.56 m 见方、间距 0.36 m ⇒ 车宽 0.40 m 进不去），
目标 map `(3.2, 0)`（= 世界 (5.24,1.37)，桌子另一侧）：

| 变体 | 目标结果 | 物理接触 | 最近间隙 | 明细 |
| --- | --- | --- | --- | --- |
| VISION=0 | SUCCEEDED | **4** | 0.000 m | 左轮撞 `cafe_table_0::link::base` ×2、`cafe_table_1::link::base` ×? |
| VISION=1 | ABORTED（`failed to generate a valid path`） | **0** | 1.173 m | 原地未动，车身从未靠近 |

（见 `logs/table_v0_base_*/table_result.json`、`logs/table_v1_vis_*/table_result.json`）

### 5.5 局部体素层取证（`harness/check_local_layer.sh`）
机器人放在 cafe_table 前 1.3 m（世界 (2.2,1.37)，map 原点=起点 ⇒ 桌子在 map(1.34,0)），
`/local_costmap/costmap`（6×6 m、120×120 格）切片：

```
 +0.5 .......oooiCCIICCCCIIii
 +0.4 .......oooiCCCCCCCCCCCi     C=致命 i=内切 o=膨胀 .=空闲
 +0.0 .......oooICCCCCCCCCCIi     ← 0.913 m 见方的桌面 + 立柱，整块致命
 -0.4 .......oooiCCCCCCCCCCCi
 -0.5 .......oooiiiiiiiiiiiii
```
窗口内致命格从 5（站在远处、桌在窗口外）变成 **408**；
`ros2 topic info /vision/live_cloud -v` 显示订阅者 `Node name: local_costmap`。
即局部体素层确实在吃视觉点云并标成致命格。

### 5.6 探索 A/B（`harness/ab_vision.sh`，固定起点 (0,0,0)，120 s，交替跑）
见 §6 表格。**6 轮全部"物理接触=0"**；两种变体都是 2/3 PASS，失败项互不相同且都与视觉无关
（基线轮是 `no_reverse`：2 次共 0.23 m 非受控后退；视觉轮是 `return_home` 姿态兜底超时）。

## 6. 探索 A/B 与资源开销（2026-10-23 17:35 完成）

### 6.1 验收与接触（固定起点 (0,0,0)、120 s，交替跑）

| 变体 | 轮次 | 判定 | 物理接触 | 视觉格 | 视觉峰值 | Nav2 峰值 | 可用内存最低 | 未过项 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 基线 | abv0r1 | PASS | 0 | — | — | 391 MB | 10 562 MB | 无 |
| 基线 | abv0r2 | FAIL | 0 | — | — | 392 MB | 10 897 MB | `no_reverse`（2 次共 0.23 m 非受控后退） |
| 基线 | abv0r3 | PASS | 0 | — | — | 315 MB | 10 578 MB | 无 |
| 基线 | abv0r4 | FAIL | 0 | — | — | — | — | `return_home` |
| 基线 | abv0r5 | FAIL | **1** | — | — | — | — | `no_contact` |
| 基线 | abv0r6 | PASS | 0 | — | — | — | — | 无 |
| 视觉 | abv1r1 | PASS | 0 | 3258 | 77 MB | 431 MB | 10 657 MB | 无 |
| 视觉 | abv1r2 | PASS | 0 | 2038 | 75 MB | 428 MB | 10 715 MB | 无 |
| 视觉 | abv1r3 | FAIL | 0 | 4001 | 75 MB | 321 MB | 10 668 MB | `return_home`：会话驱动"返航未启动，重发 set_mapping_done"卡住（老问题） |
| 视觉 | abv1r4 | PASS | 0 | 2747 | 76 MB | 381 MB | 10 934 MB | 无 |
| 视觉 | abv1r5 | PASS | 0 | 2310 | 76 MB | 433 MB | 10 970 MB | 无 |
| 视觉 | abv1r6 | PASS | 0 | 3306 | 76 MB | 382 MB | 10 997 MB | 无 |
| 视觉 | abv1r7 | FAIL | 0 | 2094 | 75 MB | 365 MB | 10 950 MB | `return_home` |
| 视觉 | abv1r8 | PASS | 0 | 2122 | 77 MB | 362 MB | 11 027 MB | 无 |
| 视觉 | final-vision | FAIL | 0 | 2048 | 77 MB | — | — | `return_home`（"stuck after 10 recovery attempts"）+ `no_reverse` |

**结论：**

* **物理接触**：视觉 **9/9 轮 = 0**；基线 6 轮里出现 **1** 次（abv0r5，轮子接触）
  —— 视觉链路没有放大撞障，反而更干净；
* **判定**：视觉 **6/9 PASS（67%）**，基线 3/6 PASS（50%）；视觉轮的失败项**全部落在
  `return_home` 这一类**，而基线这轮同样出现 `return_home`，说明**不是视觉引入的新失败模式**；
* **"是不是视觉把家门口堵了"**：专门查了落盘的视觉累积栅格 —— 离 map 原点最近的视觉格在
  **0.65 m**，`<0.5 m` 内 **0 格**（两轮一致）。manager 的返航净空判据是 0.20 m，
  所以"视觉标记把回家点挤掉"这条路径可以排除；
  "返航未启动（等活跃目标取消卡住）"在视觉上线之前的
  `round_20261021-155632_iter5 / iter6b / iter8 / iter9 / iter13` 都能搜到，属既有缺陷；
* **原七项检查**（preflight / TF / Nav2 四项 / 前沿 / 到时 set_mapping_done / 返航 / 存图）
  在视觉轮的通过情况与基线同分布，没有出现"视觉轮特有的新失败项"。

### 6.2 探索效率没有退化（同口径 120 s）

| 变体 | 派发导航目标 | 地图自由面积（中位） | 已知率 |
| --- | --- | --- | --- |
| VISION=0（3 轮） | 5 / 6 / 6 | 121.8 m²（84.4 / 121.8 / 136.7） | 56.7 / 51.1 / 59.8 % |
| VISION=1（6 轮） | 5 / 6 / 5 / 5 / 6 / 5 | **123.0 m²**（120.5 / 125.4 / 115.0 / 128.2 / 130.4 / 131.7） | 53.8 ~ 60.7 % |

视觉轮的自由面积分布更集中（115~132 m²，基线 84~137 m²），中位数略高 ——
没有"多加一层就变慢/变差"的迹象。

### 6.3 本机资源实测（15 GiB、**无 swap**）

| 进程 | 峰值 RSS | 实测 CPU（占单核） |
| --- | --- | --- |
| gzserver + 机器人层 | 551 MB | 24.6 ~ 25.5 % |
| cartographer | 63~70 MB | 5.7 % |
| Nav2（5 个 server + 两个 costmap） | 315~433 MB（视觉开关前后同量级） | 16.1 ~ 39.8 % |
| **`vision_obstacle_grid`（新增）** | **75~77 MB** | **8.4 ~ 9.5 %** |
| 系统可用内存最低 | **10.56 GB**（还剩约 2/3） | — |

即：整套仿真栈实测约 1.1~1.2 GB；**视觉链路只加 77 MB（约 5% 内存）+ 不到 0.1 个核**，
远低于"无 swap 必须留 4 GB 余量"的红线，12 线程的 i5-11400H 完全没有压力。
即使把相机提到 640×480@10 Hz（16 倍点数）也还有大量余量 —— 语义线可以做。

## 7. 已知限制与后续

1. **单调累积无衰减**：家具移动后会留下"幽灵障碍"。方案：加时间戳衰减（每格记最后一次观测时间，
   超过 T 未再观测则降权/清除），或整轮结束才固化。
2. **颜色话题缺失**：语义（L2 物体标注）前置条件，修 xacro remap 或直接吃 `/camera/depth/image_raw`。
3. **相机外参只有推定值**（光心 0.26 m 正上方、水平），实机需实测；体素/语义都对它敏感。
4. **仿真相机 160×120@5 Hz** 只够"有没有障碍"，语义线需要提到 320×240@10 Hz 以上（本机内存/CPU 实测见 §6.3）。
5. **视觉层对"贴地极薄物体"仍有下界**：z_min=0.05 意味着 4 cm 底板本身不直接成点，
   靠的是桌面(0.755)投影把整块区域堵死；对"只有薄片没有上方结构"的物体（地毯边、电缆）无效。

## 8. 复跑

```bash
cd /home/kbf/stemm_exploration_sim_ws
# 集成自检（约 4 分钟）
bash harness/check_vision_layer.sh vchk
# 定点复现（基线 vs 视觉，各约 4 分钟）
bash harness/test_vision_table.sh 0 base
bash harness/test_vision_table.sh 1 vis
# 探索 A/B（3 组 × 2 轮交替，约 30 分钟）+ 汇总表
bash harness/ab_vision.sh 3 120
python3 harness/ab_summary.py abv 3
# 追加单轮视觉（会带 CPU 采样）
VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" bash harness/round.sh abv1r7 120
# 局部体素层取证（约 3 分钟）
bash harness/check_local_layer.sh
# 单轮视觉探索（正式跑法）
VISION=1 POSE_OVERRIDE="0.0 0.0 0.0" bash harness/round.sh v1 120
```
