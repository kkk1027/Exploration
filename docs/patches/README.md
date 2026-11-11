# nav2_behavior_tree 补丁：GoalUpdater 跨目标污染修复

## 为什么需要这个补丁

`nav2_behavior_tree` 的 `GoalUpdater` 装饰器有一个上游缺陷：它用**实例成员**
`last_goal_received_` 缓存收到的 `/goal_update`，判断条件是

```cpp
if (rclcpp::Time(last_goal_received_.header.stamp) > rclcpp::Time(goal.header.stamp)) {
  goal = last_goal_received_;
}
```

该成员**永不清理**，且判断完全押在时间戳比较上。行为树里
`input_goal.header.stamp` 常常是 0，于是 `T > 0` 恒真 ⇒ **每个新目标都被替换成
历史上那一次的纠偏点** ⇒ 车本来就在那个点附近 ⇒ controller 首次
`isGoalReached()` 就命中 ⇒ 30 ms「假到达」。

实测后果（P13-5 时期，v2 五轮）：**179/226 = 79.2% 的 reached 属于这一类**，
而 inplace 未触发的五轮（offv2*）幻影数 = 0，相关性 100%。

## 修复内容

1. **换目标即清缓存**：`input_goal` 位置变化 > 5 cm 判定为新目标，
   清空 `last_goal_received_`。跨目标污染因此不可能发生，同一目标内仍正常接受纠偏。
2. **`prev` 记录输出而非输入**：因为 BT 里 `output_goal` 会写回 `{goal}` 本身
   （这样下游 `GlobalUpdatedGoal` 才能察觉目标变了并重规划），下一拍的输入
   恰好等于上一拍的输出。若 `prev` 记录输入，每两拍就会误判一次「换了新目标」
   而清空缓存，纠偏点会在「应用 → 清空」之间来回抖动。

## 应用方法

前提：已有 nav2 源码（本补丁基于 nav2 1.1.20 / ROS 2 Humble）。

```bash
cd <你的工作空间>/src/navigation2-humble/nav2_behavior_tree
git apply -p1 /path/to/docs/patches/nav2_goalupdater_fix.patch
```

`-p1` 会剥掉 `a/` `b/` 前缀，所以要在 **nav2_behavior_tree 目录内**执行。

应用后需要重新编译：

```bash
cd <你的工作空间>
colcon build --packages-select nav2_behavior_tree --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

`-DBUILD_TESTING=OFF` 是必需的：若系统未安装 `test_msgs`，
`test/plugins/action/CMakeLists.txt` 的 `find_package(test_msgs)` 会让 cmake 直接失败。

## 验证补丁是否真的生效

编译产物是**独立插件库**（不是 `libnav2_behavior_tree.so`）：

```bash
ls -l install/nav2_behavior_tree/lib/libnav2_goal_updater_node_bt_node.so
strings install/nav2_behavior_tree/lib/libnav2_goal_updater_node_bt_node.so | grep -c GoalUpdater
# 应远大于 0
```

更可靠的运行期证据是订阅者数量：

```bash
ros2 topic info /goal_update
# 必须看到 Subscription count: 1
```

**看参数值或看日志都不作数。** 踩过的坑：只改
`bt_navigator.default_nav_to_pose_bt_xml` 是无效的，因为
`stemm_cartographer_auto_mapping.launch.py` 会用
`exploration_behavior_tree` 参数把行为树显式传给 manager，绕过了
`bt_navigator` 的默认树。实测那时订阅者数 = 0，GoalUpdater 根本没加载。

## 配套的行为树

修复本身不会启用 inplace，还需要一份挂了 `GoalUpdater` 的行为树。
见 `wheeltec_ros2` 仓库：

```
src/stemm_cartographer_exploration/config/stemm_cartographer_nav_to_pose_goalupdater.xml
```

它与车端原树 `stemm_cartographer_nav_to_pose.xml` 逐节点相同，唯一差异是把
`Fallback name="ComputePathIfNeeded"` 整段包进了
`GoalUpdater(input_goal="{goal}", output_goal="{goal}")`。

## 已知副作用

启用 inplace 后，manager 的 `reached; ... error=(xy=X m)` 会**天然偏大**。
原因是**记账口径不一致**：仲裁器把目标 G 挪到附近的合格点 G*，controller 到 G*
即判 succeeded，而 manager 仍按原目标 G 记账。

实测铁证（`p13s4_g4` 第 1095 行）：

| 事件 | 时间 | 坐标 |
|---|---|---|
| 派发目标 G | 4642.00 | (9.20, 0.58)，车在 (8.00, −7.75) |
| inplace 纠偏 G→G* | 4660.60 | → **(6.45, 1.62)**，距离 2.95 m |
| 到达 | 4679.46 | 车在 **(6.55, 1.69)** |

- 车到 **G\*** 仅 0.122 m（正常收敛）
- manager 记录误差 **2.875 m**（按 G 算的）
- 存活时长 **37.5 秒**（不是幻觉的 30 ms）

**所以「到达误差」这一指标在启用 inplace 前后不可直接比较。**
判断真伪用 `harness/s4_ghost_audit.py`：它区分「inplace 记账」与「30 ms 幻觉」。
实测 10 轮 46 条到达，幻觉 0 条。

## 补丁来源

- 生成日期：2026-11-11
- 基线版本：nav2 1.1.20（ROS 2 Humble 二进制包 `/opt/ros/humble`）
- 改动文件：
  - `plugins/decorator/goal_updater_node.cpp`
  - `include/nav2_behavior_tree/plugins/decorator/goal_updater_node.hpp`
