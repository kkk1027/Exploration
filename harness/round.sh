#!/usr/bin/env bash
# round.sh —— 探索建图单轮运行器（确定性启动/收尾 + 六项验收取证）
# 用法: round.sh <标签> <探索秒数> [启动后等待秒数(仅空跑用)]
# 产物: runs/<时间戳>-<标签>.jsonl 摘要 + logs/round_<时间戳>_<标签>/launch.log
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
# ---- DDS 只走回环（2026-10-26）----
# 事故：机器切到手机热点（172.20.10.5/28）之后，组播发现时好时坏 ——
#   `ros2 node list`=0 但 `service list`=200+、`ros2 param set` 全部超时、
#   TF 消息收不到（RViz 里"TF 断了"）、guard 判 TF 不健康把整栈拆掉。
# 本仓库的仿真全在本机，所以强制 ROS_LOCALHOST_ONLY=1（走 shm/loopback），
# 彻底不依赖网卡。需要跨机看 RViz 时用 SIM_REMOTE=1 关掉本保护。
if [ "${SIM_REMOTE:-0}" != "1" ]; then
  export ROS_LOCALHOST_ONLY=1
fi
source /home/kbf/stemm_sim_ws/domain_guard.sh || exit 9
source /opt/ros/humble/setup.bash
source /home/kbf/stemm_sim_ws/install/setup.bash
source /home/kbf/stemm_explore_ws/install/setup.bash
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash
# S4（2026-11-11）：最后 source，让 wheeltec_ros2 里重编译的 nav2_behavior_tree
# （含 GoalUpdater 跨目标污染修复）覆盖 /opt/ros/humble 的系统版本。
# 该工作空间只编了 nav2_behavior_tree 一个包，不会遮蔽其他包。
if [ -f /home/kbf/wheeltec_ros2/install/setup.bash ]; then
  source /home/kbf/wheeltec_ros2/install/setup.bash
fi
# ---- 干掉遗留的 ros2cli daemon（2026-10-26，第二个真根因）----
# 现象：`ros2 node list` 空、`ros2 param set` 全超时、`ros2 param dump` 全空，
# 但 rclpy 节点之间订阅/服务完全正常 —— 因为 daemon 是个常驻进程，
# 它在 **没有 ROS_LOCALHOST_ONLY=1 的环境**里起过（切热点之前），
# 之后所有 CLI 命令都复用这个"看不见回环节点"的 daemon。
# 实测：`ros2 daemon stop` 之后同一命令立刻能列出 /probe_node。
# 由此连锁出来的假故障：patch_sim_time 每项"超时"（参数没落地）、
# param_audit 恒 0/8（被判 invalid）、起栈就绪标记迟迟不出现。
# 这里每轮开跑前先停 daemon，让 CLI 用本轮环境另起一个。
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
# ---- 并发保护（2026-10-26）：同一时刻只允许一轮仿真 ----
# 事故：手动启动的轮次和批次轮次撞在一起（同一 ROS_DOMAIN_ID / 同一个 gzserver），
# 两边的指标都会变脏。拿不到锁就直接退出，且不改动任何文件。
LOCK=/tmp/stemm_round.lock
exec 9>"$LOCK"
if ! flock -n 9; then
  echo "✗ 已有另一轮仿真在跑（lock=$LOCK）；本轮放弃，避免数据互相污染" >&2
  exit 11
fi


TAG="${1:-run}"; HOLD="${3:-0}"
# 探索秒数必须是 DOUBLE（session_driver 把 explore_sec 声明成 double，传 120 会 InvalidParameterType）
EXPLORE=$(python3 -c "print(float('${2:-120.0}'))")
EXPLORE_F=$(awk -v v="$EXPLORE" 'BEGIN{printf "%.4f", v}')
BASE=/home/kbf/stemm_exploration_sim_ws
TS=$(date +%Y%m%d-%H%M%S)
OUT="$BASE/logs/round_${TS}_${TAG}"; mkdir -p "$OUT"
J="$BASE/runs/${TS}-${TAG}.jsonl"
# 返航"等精确终态"的宽限（P0-1，用户要求暂时放宽）：>0 时用可回退注入改车端 launch，0=原厂 6 s
GRACE_SEC="${GRACE_SEC:-22.0}"
# 方案A 视觉层开关（VISION=1 时注入 nav2 视觉图层 + 起视觉障碍源；0=纯雷达基线）
# 2026-10-27 接入：**默认改为 1**。原因：雷达单平面看不到（健身器材/低矮家具）的物体
# 只有视觉能补；默认 0 会让"雷达盲区"在探索基准里长期缺席，指标无法反映真实通行性。
VISION="${VISION:-1}"
# 全局层视觉源：accum=累积云+不清除（默认，实测能真正挡住路径）/ livemark=实时云+不清除
# / live=实时云+清除（旧行为：实测路径会从视觉障碍的格子缝里穿过去，只作对照）
VISION_MODE="${VISION_MODE:-${VISION_GLOBAL:-accum}}"
# 候选后验门（2026-10-27）：候选是在当时的地图上选的，扫到之后同一位置可能已是高代价。
#   COST_GATE=1 → 仲裁器下发前用当前全局代价地图复核落点（拒 + 黑名单）
#   GOAL_GATE=1 → 把 manager 的 frontier_goal_topic 接到 /validated_frontier_goal，
#                 让仲裁器成为唯一派发源（否则仲裁器的否决全在死分支上）
COST_GATE="${COST_GATE:-0}"
# 方向黏性：默认放开（用户 2026-10-27 决定），转角只在仲裁器里做软惩罚
TURN_GATE="${TURN_GATE:-off}"
GOAL_GATE="${GOAL_GATE:-1}"
MEMLOG="$OUT/mem.jsonl"
# 环境变量兜底：仲裁器的 timeline 路径（launch 参数偶发丢失时用这个）
export STEMM_TIMELINE_LOG="$OUT/timeline.jsonl"

# 内存采样：本机 15 GiB 且 **无 swap**，视觉链路必须证明"加得起"
mem_sample() {
  local avail total=0 pid r
  avail=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo 2>/dev/null)
  rss_of() {
    local sum=0 pp rr
    for pp in $(pgrep -f "$1" 2>/dev/null); do
      rr=$(awk '/VmRSS/{print $2}' /proc/$pp/status 2>/dev/null)
      [ -n "$rr" ] && sum=$((sum + rr/1024))
    done
    echo "$sum"
  }
  # CPU：同一批进程的累计 tick（utime+stime），由汇总脚本按时间差换算成"几个核"
  cpu_of() {
    local sum=0 pp
    for pp in $(pgrep -f "$1" 2>/dev/null); do
      [ -r "/proc/$pp/stat" ] || continue
      sum=$((sum + $(awk '{print $14+$15}' "/proc/$pp/stat" 2>/dev/null || echo 0)))
    done
    echo "$sum"
  }
  printf '{"t":%s,"avail_mb":%s,"gz_mb":%s,"vision_mb":%s,"nav2_mb":%s,"carto_mb":%s,"cpu_gz":%s,"cpu_vision":%s,"cpu_nav2":%s,"cpu_carto":%s}\n' \
    "$(date +%s)" "${avail:-0}" "$(rss_of 'gzserver')" "$(rss_of 'vision_obstacle_grid')" \
    "$(rss_of 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower')" \
    "$(rss_of 'cartographer_node')" \
    "$(cpu_of 'gzserver')" "$(cpu_of 'vision_obstacle_grid')" \
    "$(cpu_of 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower')" \
    "$(cpu_of 'cartographer_node')" >> "$MEMLOG"
}
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/harness.log"; }
rec() { printf '{"ts":"%s","tag":"%s","%s"}\n' "$(date -Iseconds)" "$TAG" "$1" >> "$J"; }

# ---- 1) 启动前断言：域 42 零节点 + 无 gzserver ----
proc_residue() { pgrep -af "waypoint_follower|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|cartographer_node|occupancy_grid|stemm_cartographer|robot_state_publisher|ekf_node|imu_bias_shim|scan_deadzone_relay|async_slam_toolbox|nav2_|sim_clock_resampler" 2>/dev/null | grep -vE "harness/(round|sweep)\.sh|grep|pgrep" | awk '{print $1}' | grep -vxE "$$|$PPID" | tr '\n' ' '; }
NODES=$(timeout 10 ros2 node list 2>/dev/null | wc -l)
GZ=$(pgrep -cx gzserver)
PRES=$(proc_residue)
say "启动前：ROS 节点数=$NODES（含DDS僵尸，仅记录）  gzserver=$GZ  进程残留=[${PRES}]"
rec "\"check\":\"pre_launch\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
if [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; then
  say "存在残留，先清理"
  for p in $(proc_residue); do kill -TERM "$p" 2>/dev/null; done
  sleep 2
  for p in $(proc_residue); do kill -9 "$p" 2>/dev/null; done
  pkill -x gzserver 2>/dev/null; sleep 4
  NODES=$(timeout 10 ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver); PRES=$(proc_residue)
  say "清理后：ROS 节点数=$NODES  gzserver=$GZ  进程残留=[${PRES}]"
  rec "\"check\":\"post_cleanup\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${PRES}\""
  { [ -n "$PRES" ] || [ "$GZ" -gt 0 ]; } && { say "清理不彻底，放弃本轮"; rec '"verdict":"FAIL_preclean"'; exit 10; }
fi

rm -f /home/kbf/stemm_sim_ws/maps/explore_latest.pgm /home/kbf/stemm_sim_ws/maps/explore_latest.yaml
T_START=$(date +%s)

# 随机起点（5 个候选，房间原点附近的安全偏移位姿）
# 出生点：house world 里"略靠墙"的点（离最近墙面 0.40 m、周围 1 m 自由率 0.67、
# 四个方向里 3 个能直行 1.2 m 以上）。原房间的坐标在这里是"屋子外面"，所以换掉。
# 默认世界（放在走廊阈值判定之前：走廊默认值要按世界取）
DEFAULT_WORLD=$(ros2 pkg prefix wheeltec_two_wheel_model 2>/dev/null)/share/wheeltec_two_wheel_model/world/aws_small_house.world
POSES=("1.00 5.10 -1.5708" "0.80 5.10 -1.5708" "1.20 5.10 -1.5708" "1.00 5.05 -1.5708" "0.90 5.15 -1.5708")
# 浮点参数统一格式化成 "x.y"：launch_ros 会把 "10" 解析成 int，
# 而节点 declare 的默认值是 float → 类型不匹配会让节点当场崩（vx1 就这么废了 7 轮）。
as_float() { awk -v v="$1" 'BEGIN{printf "%.4f", v}'; }
VETO_CD_F=$(as_float "${VETO_CD:-10.0}")
# 通行判据**不随场景切换**（2026-10-25 用户要求）：阈值只由车体尺寸+误差决定，
# 由仲裁器内部的 pass_width_m 给出（车宽 0.47 + 2×0.215 ≈ 0.90 m）。
# CORRIDOR_W 仅作为旧接口保留（默认等于 pass_width），不再按 World 分支取值。
CORRIDOR_W_F=$(as_float "${CORRIDOR_W:-0.90}")
GATE_OCC_F=$(as_float "${GATE_OCC:-0.30}")
VETO_OCC_F=$(as_float "${VETO_OCC:-0.30}")
# 默认世界：aws_robomaker_small_house（2026-10-25 起替代原开阔房间）
# 要回到原房间：WORLD=<...>/custom_room_2ms.world bash harness/round.sh ...
POSE=${POSE_OVERRIDE:-${POSES[$((RANDOM % ${#POSES[@]}))]}}
set -- $POSE
# 三个 spawn 值必须带小数点：POSE_OVERRIDE="0 0 0" 会被 launch_ros 解析成 int，
# 而 launch 里它们声明的默认值是 0.0(float) → 类型不匹配会让节点当场崩。
SX=$(as_float "$1"); SY=$(as_float "$2"); SYAW=$(as_float "$3")
say "本轮随机起点：x=$SX y=$SY yaw=$SYAW"
rec "\"check\":\"pose\",\"x\":$SX,\"y\":$SY,\"yaw\":$SYAW"

# ---- 2) 启动（记录真实 PGID） ----
# P0 变体开关（默认=改造后：关盲退 + 开脱困监督）
DISABLE_RETREAT="${DISABLE_RETREAT:-1}"
SUPERVISOR="${SUPERVISOR:-true}"
CAR_ESCAPE="${CAR_ESCAPE:-$([ "${DISABLE_CAR_ESCAPE:-1}" = "1" ] && echo false || echo true)}"
say "P0 变体：disable_retreat=$DISABLE_RETREAT escape_supervisor=$SUPERVISOR"
rec "\"check\":\"variant\",\"disable_retreat\":$DISABLE_RETREAT,\"supervisor\":\"$SUPERVISOR\""

# ---- 方案A：nav2 视觉图层注入（幂等、可回退；teardown 后自动撤销）----
# ---- P0-1：返航姿态宽限注入（可回退；teardown 撤销）----
if [ "$(python3 -c "print(1 if float('$GRACE_SEC')>0 else 0)")" = "1" ]; then
  # V2-① 的最终形态（2026-10-25，docs/P7 第 27/28 节）：
  # 12 s 太长（manager 空等、车静止、返航退化），但在 max_candidates=14 + veto 之后
  # **4 s** 恰好够仲裁在同一拍里多试十几个候选：
  #   20 轮对照：manager 自派 42%→11%、静止超时 1.45→0.55 次/轮、归因率 22%→9%，
  #   全绿 19/20=95% 与基线完全相同、返航失败 0。
  RRT_WAIT="${RRT_WAIT:-4}"
  FT_ARG=""
  # GOAL_GATE=1：manager 只认"合并器"输出（仲裁合格优先 + RRT 兜底）。
  # 直接接 /validated_frontier_goal 会断供：实测一轮里连续 93 s 停在 waiting_for_rrt_goal。
  if [ "$GOAL_GATE" = "1" ]; then FT_ARG="--frontier-topic /merged_frontier_goal"; fi
  if [ "$TURN_GATE" = "off" ]; then FT_ARG="$FT_ARG --turn-gate off"; fi
  python3 harness/patch_car_launch.py --apply --grace "$GRACE_SEC" --rrt-wait "$RRT_WAIT" --front-stop "${FRONT_STOP:-0}" $FT_ARG 2>&1 | tee -a "$OUT/harness.log"
  if [ "$GOAL_GATE" = "1" ]; then
    say "★ 候选后验接线：manager 的 frontier_goal_topic → /merged_frontier_goal（仲裁优先 ttl=${MERGE_TTL:-5.0}s + RRT 兜底）"
    rec '"check":"goal_gate","enabled":true'
  fi
  say "返航姿态宽限：${GRACE_SEC}s（原厂 6 s；GRACE_SEC=0 可关）｜RRT 等待窗口：${RRT_WAIT:-0}s｜front_stop=${FRONT_STOP:-原厂0.65}"
  trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/patch_car_launch.py --restore >/dev/null 2>&1; python3 /home/kbf/stemm_exploration_sim_ws/harness/nav_params.py --restore >/dev/null 2>&1; python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM

# ---- P9 ①②: Nav2 恢复动作 / 重规划节拍（两项独立开关，逐字节可回退）----
# RECOVERY_BT=replan → 只提重规划节拍(1.0 Hz)，恢复动作仍是原厂 Wait(2)
# RECOVERY_BT=spin   → 额外把 Wait(2) 换成受限 Spin（会改变位姿；实测在墙角顶死时
#                      会拿"物理接触"换位移，必须能和只提节拍分开验证）
# RECOVERY_BT=off    → 显式还原成原厂（Wait + 0.3 Hz）；不设=不动这个文件
RECOVERY_BT="${RECOVERY_BT:-none}"
case "$RECOVERY_BT" in
  replan)
    python3 harness/patch_recovery_bt.py --apply --spin off --replan-hz "${REPLAN_HZ:-1.0}" --backup "${BACKUP:-0}" 2>&1 | tee -a "$OUT/harness.log"
    rec '"check":"recovery_bt","mode":"replan"';;
  spin)
    python3 harness/patch_recovery_bt.py --apply --spin on --spin-dist "${SPIN_DIST:-0.6}" --spin-time "${SPIN_TIME:-6.0}" --replan-hz "${REPLAN_HZ:-1.0}" 2>&1 | tee -a "$OUT/harness.log"
    rec '"check":"recovery_bt","mode":"spin"';;
  off)
    python3 harness/patch_recovery_bt.py --restore 2>&1 | tee -a "$OUT/harness.log"
    rec '"check":"recovery_bt","mode":"off"';;
  *) : ;;
esac
say "Nav2 恢复动作/节拍：RECOVERY_BT=$RECOVERY_BT（none 表示不加补丁）"
fi

say "视觉层 VISION=$VISION（global-mode=$VISION_MODE）｜候选后验 COST_GATE=$COST_GATE GOAL_GATE=$GOAL_GATE｜修复 REPAIR_ON=${REPAIR_ON:-1} R=${REPAIR_R:-1.20} N=${REPAIR_N:-24} KEEP=${REPAIR_KEEP:-0.30} MINUNK=${REPAIR_MIN_UNK:-20}"
rec "\"check\":\"vision\",\"enabled\":$VISION,\"global_mode\":\"$VISION_MODE\",\"cost_gate\":$COST_GATE,\"goal_gate\":$GOAL_GATE"
trap 'python3 /home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py --restore >/dev/null 2>&1' EXIT INT TERM
  # 一次性固定参数决策：速度上限 0.35 m/s（VX_MAX=0 表示不注入、用车端原值 0.5）
  # 依据：P0 4v4（docs/P8 第 4.2 节）——卡住事件 8→5、接触 0、全绿 3/4→4/4。
  VX_MAX="${VX_MAX:-0.35}"
  if [ -n "${VX_MAX:-}" ] && [ "${VX_MAX}" != "0" ]; then
    NAV_ARGS=(--vx-max "$(as_float "${VX_MAX:-0.35}")")
    if [ "${UNIFY:-0}" = "1" ]; then
      NAV_ARGS+=(--footprint "${UNIFY_FP:-[[-0.18,-0.485],[-0.18,0.485],[0.32,0.485],[0.32,-0.485]]}"
                 --pad "$(as_float "${PAD_UNIFY:-0.0}")"
                 --vx-min "-$(as_float "${VX_MIN:-0.08}")"
                 --allow-unknown false)
      say "统一口径：footprint=${UNIFY_FP:-0.97m宽} padding=${PAD_UNIFY:-0.0} vx_min=-$(as_float "${VX_MIN:-0.08}") allow_unknown=false"
    fi
    [ -n "${VX_MIN:-}" ] && NAV_ARGS+=(--vx-min "$VX_MIN")
    # 只改**全局**膨胀半径（局部保持原厂 0.45）：车自身格被判"起点致命"是因为
    # 全局 inflation 0.40 ≥ 车体到障碍的距离；降到 0.30 后车贴墙时不再整格抬升。
    [ -n "${INFL_R:-}" ] && NAV_ARGS+=(--infl-radius "$(as_float "$INFL_R")")
    # Smac 降采样（压搜索规模，治 'exceeded maximum iterations'）
    [ -n "${SMAC_DS:-}" ] && [ "${SMAC_DS}" != "1" ] && NAV_ARGS+=(--smac-downsample "$SMAC_DS")
    # 运动模型/倒车代价（P9：允许先验后的有限倒车 ⇒ REEDS_SHEPP）
    [ -n "${MOTION:-}" ] && NAV_ARGS+=(--motion "$MOTION")
    [ -n "${REV_PEN:-}" ] && NAV_ARGS+=(--reverse-penalty "$(as_float "$REV_PEN")")
    # S4（2026-11-11）：切到挂了 GoalUpdater 的探索行为树。
    # ★ 注意不能用 nav_params 的 --bt-xml：那个改的是 bt_navigator 的
    #   default_nav_to_pose_bt_xml，而 manager 派发时显式传
    #   exploration_behavior_tree 参数，会盖掉它（实测订阅者仍为 0）。
    # 前置条件：nav2_behavior_tree 已含跨目标污染修复（2026-11-11 补丁）。
    # 配套 VETO_INPLACE=1，后置否决才会走「原地改向」而不是「取消重派」。
    if [ "${BT_GOALUPDATER:-0}" = "1" ]; then
      BT_ARG=" exploration_behavior_tree:=${BT_XML_PATH:-/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/stemm_cartographer_nav_to_pose_goalupdater.xml}"
      say "S4：探索树切到 stemm_cartographer_nav_to_pose_goalupdater.xml（挂 GoalUpdater）"
      # ★ 自动带上 VETO_INPLACE 与经验证可用的 inplace 判据。
      #   默认判据（occ<0.15 / esc>=4 / width>=0.70）比后置否决门槛（occ<0.30 /
      #   esc>=3）严得多，实测几乎找不到合格点 → inplace 从不触发（s4v2 实测 0 次）。
      #   下列值经 s4v3 实测可正常触发（inplace=2 次、migrate=0 次），
      #   且判断依据与否决门槛同口径，不会把车导向更差的位置。
      if [ "${VETO_INPLACE:-0}" = "1" ]; then
        INPLACE_OCC="${INPLACE_OCC:-0.25}"
        INPLACE_ESC="${INPLACE_ESC:-3}"
        INPLACE_W="${INPLACE_W:-0.58}"
        INPLACE_R="${INPLACE_R:-5.0}"
        INPLACE_MAX="${INPLACE_MAX:-3}"
        INPLACE_DISP="${INPLACE_DISP:-0.60}"
      fi
    else
      BT_ARG=""
    fi
    if [ "${COST_INFL:-0}" = "1" ]; then
      NAV_ARGS+=(--infl-radius "${GLOBE_R:-1.00}" --infl-scaling "${GLOBE_S:-1.2}" --pad "${PAD:-0.10}")
    fi
    if [ "${PLANNER_PATCH:-0}" = "1" ]; then
      NAV_ARGS+=(--allow-unknown "${ALLOW_UNKNOWN:-false}" --motion "${MOTION_MODEL:-REEDS_SHEPP}"
                 --reverse-penalty "${REV_PENALTY:-3.0}")
    fi
    python3 harness/nav_params.py --apply "${NAV_ARGS[@]}" 2>&1 | tee -a "$OUT/harness.log"
    NAV_RC=${PIPESTATUS[0]}
    if [ "$NAV_RC" != "0" ]; then
      say "✗ nav2 参数注入失败，放弃本轮"
      python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
      rec '"verdict":"FAIL_nav_params"'
      exit 13
    fi
  fi

  # ★★ 2026-10-27 顺序修正（当天最重要的 bug）★★
  # nav_params.py 按 pristine 快照**整份重写** nav2 参数文件；如果视觉图层先注入，
  # 就会被这次重写**整块抹掉** ⇒ 探索建图的 nav2 从来没有 vision_* 层
  # ⇒ 视觉障碍不进全局代价地图 ⇒ 规划路径横穿"雷达看不见的视觉障碍"。
  # nav_only 不跑 nav_params，所以那边没有这个症状（用户实测对比即由此而来）。
  # 因此顺序必须是：nav_params → 视觉注入 → 回读校验。
  if [ "$VISION" = "1" ]; then
    python3 harness/vision_nav_params.py --apply --global-mode "$VISION_MODE" 2>&1 | tee -a "$OUT/harness.log"
    VISION_APPLY_RC=${PIPESTATUS[0]}
    if [ "$VISION_APPLY_RC" != "0" ]; then
      say "✗ 视觉图层注入失败，放弃本轮（参数文件未被改动）"
      rec '"verdict":"FAIL_vision_params"'
      exit 11
    fi
    VISION_CHK=$(python3 harness/vision_nav_params.py --check 2>&1)
    if ! echo "$VISION_CHK" | grep -q 'local_costmap.plugins = .*vision_voxel_layer' \
       || ! echo "$VISION_CHK" | grep -q 'global_costmap.plugins = .*vision_obstacle_layer'; then
      say "✗ 视觉图层回读校验失败：costmap plugins 里没有视觉图层，放弃本轮"
      echo "$VISION_CHK" | tail -4 | tee -a "$OUT/harness.log"
      python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
      rec '"verdict":"FAIL_vision_layers"'
      exit 12
    fi
    say "✓ 视觉图层回读校验通过（已确认在 nav_params 之后落盘）"
    echo "$VISION_CHK" | grep -E '\.plugins = ' | tee -a "$OUT/harness.log"
    rec '"check":"vision_layers","ok":true'
  else
    python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
    python3 harness/patch_car_launch.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
  fi


# ★ 注意：下面这一行里**绝不能写行内 # 注释** —— 那个字符串是 `bash -c "..."` 的执行体，
# 行内的 # 会被内层 bash 当成注释，把其后的 watchdog_log / nudge_log /
# candidate_probe_log / timeline_log 全部吞掉（实测仲裁器 probe_log 运行期为 ''，
# candidate_probe.jsonl 从 10-26 起再没生成过；timeline 只是靠 STEMM_TIMELINE_LOG 兜底才还在）。
setsid bash -c "echo \$\$ > '$OUT/pgid'; exec ros2 launch stemm_cartographer_exploration_sim sim_explore.launch.py$BT_ARG explore_sec:=$EXPLORE_F world:=${WORLD:-$DEFAULT_WORLD} spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW disable_retreat:=$DISABLE_RETREAT escape_supervisor:=$SUPERVISOR car_escape:=$CAR_ESCAPE rviz:=${RVIZ:-false} gazebo_gui:=${GUI:-false} stack_delay:=$(as_float "${STACK_DELAY:-0.0}") scoring:=${SCORING:-v1} risk:=$([ "${RISK:-1}" = "1" ] && echo true || echo false) risk_dist_half:=$(as_float "${RISK_HALF:-4.5}") risk_eval_n:=${RISK_EVAL_N:-6} route_gate_soft:=$([ "${ROUTE_GATE_SOFT:-1}" = "1" ] && echo true || echo false) risk_w_soft:=$(as_float "${RISK_W_SOFT:-900.0}") raw_score:=$([ "${RAW_SCORE:-1}" = "1" ] && echo true || echo false) raw_max_dist:=$(as_float "${RAW_MAX_DIST:-8.0}") escape_allow_narrow:=$([ "${ESC_ALLOW_NARROW:-1}" = "1" ] && echo true || echo false) escape_relax_turn:=$([ "${ESC_RELAX_TURN:-1}" = "1" ] && echo true || echo false) verdict:=$([ "${VERDICT:-1}" = "1" ] && echo true || echo false) verdict_ttl:=$(as_float "${VERDICT_TTL:-6.0}") verdict_radius:=$(as_float "${VERDICT_R:-0.45}") verdict_force_after:=$(as_float "${VERDICT_FORCE:-15.0}") raw_min_esc_dirs:=${RAW_MIN_ESC:-3} raw_require_narrow_free:=$([ "${RAW_NARROW_FREE:-1}" = "1" ] && echo true || echo false) veto:=$([ "${VETO:-true}" = "1" ] || [ "${VETO:-true}" = "true" ] && echo true || echo false) plan_guard_log:=$([ "${PLAN_GUARD_LOG:-0}" = "1" ] && echo true || echo false) plan_guard_veto:=$([ "${PLAN_GUARD_VETO:-0}" = "1" ] && echo true || echo false) failfast:=$([ "${FAILFAST:-0}" = "1" ] && echo true || echo false) failfast_window:=$(as_float "${FF_WINDOW:-20.0}") failfast_drift:=$(as_float "${FF_DRIFT:-0.20}") failfast_front:=$(as_float "${FF_FRONT:-0.60}") failfast_confirm:=${FF_CONFIRM:-2} failfast_ttl:=$(as_float "${FF_TTL:-60.0}") failfast_hard_radius:=$(as_float "${FF_HARD_R:-1.2}") failfast_zone_radius:=$(as_float "${FF_ZONE_R:-2.0}") failfast_max_actions:=${FF_MAX:-8} failfast_escalate:=$(as_float "${FF_ESC:-40.0}") failfast_comp_radius:=$(as_float "${FF_COMP_R:-6.0}") return_wait_goal_sec:=$(as_float "${RWG_SEC:-20.0}") veto_max:=${VETO_MAX:-6} veto_cooldown:=$VETO_CD_F veto_migrate:=$([ "${VETO_MIGRATE:-1}" = "1" ] && echo true || echo false) veto_migrate_mode:=${VETO_MIGRATE_MODE:-random} veto_inplace:=$([ "${VETO_INPLACE:-0}" = "1" ] && echo true || echo false) inplace_radius_m:=$(as_float "${INPLACE_R:-3.5}") inplace_min_width_m:=$(as_float "${INPLACE_W:-0.70}") inplace_min_robot_m:=$(as_float "${INPLACE_RB:-0.50}") inplace_only_beyond_m:=$(as_float "${INPLACE_OB:-0.0}") inplace_max_occ_frac:=$(as_float "${INPLACE_OCC:-0.15}") inplace_min_esc_dirs:=${INPLACE_ESC:-4} inplace_max:=${INPLACE_MAX:-1} inplace_cooldown_sec:=$(as_float "${INPLACE_CD:-3.0}") inplace_min_disp_m:=$(as_float "${INPLACE_DISP:-1.00}") inplace_fallback_cancel:=$([ "${INPLACE_FALLBACK:-1}" = "1" ] && echo true || echo false) veto_migrate_r:=${VETO_MIGRATE_R:-2.0} veto_migrate_n:=${VETO_MIGRATE_N:-32} veto_migrate_move:=${VETO_MIGRATE_MOVE:-1.0} gate:=${GATE:-false} corridor_width:=$CORRIDOR_W_F max_candidates:=${MAX_CAND:-14} retreat_max:=${RETREAT_MAX:-2} retreat_interval:=$(as_float "${RETREAT_IVL:-8.0}") pass_width:=$(as_float "${PASS_W:-0.90}") narrow_floor:=$(as_float "${NARROW_FLOOR:-0.58}") narrow_len_max:=$(as_float "${NARROW_LEN_MAX:-1.00}") unknown_span_max:=$(as_float "${UNK_SPAN_MAX:-0.35}") unknown_block:=$([ "${UNKNOWN_BLOCK:-0}" = "1" ] && echo true || echo false) width_unknown_block:=$([ "${WIDTH_UNK_BLOCK:-0}" = "1" ] && echo true || echo false) noexit_enabled:=$([ "${NOEXIT:-0}" = "1" ] && echo true || echo false) noexit_pinch_m:=$(as_float "${NOEXIT_PINCH:-0.752}") noexit_short_m:=$(as_float "${NOEXIT_SHORT:-0.35}") noexit_retreat_m:=$(as_float "${NOEXIT_RETREAT:-1.20}") noexit_turn_m:=$(as_float "${NOEXIT_TURN:-0.81}") noexit_retreat_need_m:=$(as_float "${NOEXIT_RETREAT_NEED:-1.00}") noexit_turn_hold_m:=$(as_float "${NOEXIT_TURN_HOLD:-0.30}") supervisor_rotate_min_range:=$(as_float "${SUP_ROT_MIN:-0.34}") supervisor_advance_stop_range:=$(as_float "${SUP_ADV_STOP:-0.34}") supervisor_explore_reverse:=$([ "${SUP_EXPLORE_REV:-0}" = "1" ] && echo true || echo false) supervisor_explore_reverse_max_m:=$(as_float "${SUP_EXPLORE_REV_M:-0.15}") supervisor_explore_reverse_speed:=$(as_float "${SUP_EXPLORE_REV_V:-0.06}") supervisor_explore_reverse_max_count:=${SUP_EXPLORE_REV_N:-4} supervisor_explore_reverse_front:=$(as_float "${SUP_EXPLORE_REV_FRONT:-0.70}") supervisor_explore_reverse_block_sec:=$(as_float "${SUP_EXPLORE_REV_BLOCK:-6.0}") supervisor_explore_reverse_takeover:=$([ "${SUP_EXPLORE_REV_TAKEOVER:-1}" = "1" ] && echo true || echo false) supervisor_explore_reverse_rear_clear:=$(as_float "${SUP_EXPLORE_REV_REAR:-0.40}") supervisor_contact_reverse:=$([ "${CONTACT_REV:-0}" = "1" ] && echo true || echo false) supervisor_contact_reverse_window:=$(as_float "${CONTACT_REV_WIN:-5.0}") supervisor_contact_reverse_max:=${CONTACT_REV_MAX:-3} supervisor_contact_reverse_cool:=$(as_float "${CONTACT_REV_COOL:-3.0}") supervisor_contact_reverse_front:=$(as_float "${CONTACT_REV_FRONT:-0.90}") contact_fail:=$([ "${CONTACT_FAIL:-0}" = "1" ] && echo true || echo false) contact_fail_max:=${CONTACT_FAIL_MAX:-6} contact_fail_cooldown:=$(as_float "${CONTACT_FAIL_COOL:-3.0}") turn_penalty_k:=${TURN_PEN_K:-60.0} near_first:=$([ "${NEAR_FIRST:-1}" = "1" ] && echo true || echo false) near_first_r:=${NEAR_FIRST_R:-3.0} novelty_w:=$(as_float "${NOVELTY_W:-0.30}") novelty_r:=$(as_float "${NOVELTY_R:-2.0}") repeat_ff:=$(as_float "${REPEAT_FF:-0.90}") repeat_pen:=$(as_float "${REPEAT_PEN:-250.0}") near_first_band:=$(as_float "${NEAR_FIRST_BAND:-0.10}") goal_merge:=$([ "$GOAL_GATE" = "1" ] && echo true || echo false) merge_ttl:=${MERGE_TTL:-5.0} merge_raw_period:=${MERGE_RAW_PERIOD:-0.5} merge_dedupe_r:=${MERGE_DEDUPE_R:-0.0} merge_dedupe_sec:=${MERGE_DEDUPE_SEC:-0.0} merge_log:=$OUT/merge.jsonl cost_gate:=$([ "$COST_GATE" = "1" ] && echo true || echo false) cost_gate_hard:=${COST_GATE_HARD:-99} cost_gate_disc:=${COST_GATE_DISC:-0.35} cost_gate_free:=${COST_GATE_FREE:-0.80} cost_repair:=$([ "${REPAIR_ON:-1}" = "1" ] && echo true || echo false) cost_repair_radius:=${REPAIR_R:-1.20} cost_repair_samples:=${REPAIR_N:-24} cost_repair_keep_unknown:=${REPAIR_KEEP:-0.30} cost_repair_min_unknown:=${REPAIR_MIN_UNK:-20} cost_repair_verify:=$([ "${REPAIR_VERIFY:-1}" = "1" ] && echo true || echo false) cost_repair_tries:=${REPAIR_TRIES:-3} cost_repair_tick_cap:=${REPAIR_TICK_CAP:-6} cost_repair_fallback:=$([ "${REPAIR_FALLBACK:-1}" = "1" ] && echo true || echo false) corridor_block:=$([ "${CORR:-0}" = "1" ] && echo true || echo false) corridor_radius:=$(as_float "${CORR_R:-0.35}") corridor_ttl:=$(as_float "${CORR_TTL:-30.0}") corridor_ttl_max:=$(as_float "${CORR_TTL_MAX:-180.0}") probe_log:=$OUT/probe.jsonl supervisor_log:=$OUT/supervisor.jsonl vision:=$VISION vision_decay_sec:=$(as_float "${VISION_DECAY:-0.0}") vision_accum_decay_sec:=$(as_float "${VISION_ACCUM_DECAY:-$([ "$VISION_MODE" = "accum" ] && echo 120.0 || echo -1.0)}") vision_low_accum_decay_sec:=$(as_float "${VISION_LOW_DECAY:-$([ "$VISION_MODE" = "lowaccum" ] && echo 120.0 || echo 0.0)}") vision_dump:=$OUT/vision_grid watchdog_log:=$OUT/watchdog.jsonl nudge_log:=$OUT/nudge.jsonl candidate_probe_log:=$OUT/candidate_probe.jsonl timeline_log:=$OUT/timeline.jsonl ureg_enabled:=$([ "${UREG_ON:-1}" = "1" ] && echo true || echo false) ureg_radius:=$(as_float "${UREG_R:-0.50}") ureg_min_cells:=${UREG_MIN:-40} pool_probe_n:=${POOL_PROBE_N:-12} pool_probe_log:=$([ "${POOL_PROBE:-1}" = "1" ] && echo "$OUT/cand_pool.jsonl" || echo "") frame_far_first:=$([ "${FRAME_FAR:-0}" = "1" ] && echo true || echo false) frame_depth_min:=$(as_float "${FRAME_DEPTH:-0.0}") frame_supply_empty_ticks:=${FRAME_SUPPLY_EMPTY:-3} frame_stall_ticks:=${FRAME_STALL:-8} frame_growth_cells:=${FRAME_GROWTH:-400} frame_regions_max:=${FRAME_REGS:-6} frame_hysteresis:=$(as_float "${FRAME_HYST:-20.0}") frame_regrow_cells:=${FRAME_REGROW:-2500} frame_deny_limit:=${FRAME_DENY_LIMIT:-5} far_admit:=$([ "${FAR_ADMIT:-1}" = "1" ] && echo true || echo false) far_admit_dist:=$(as_float "${FAR_ADMIT_D:-5.0}") far_admit_width:=$(as_float "${FAR_ADMIT_W:-0.0}") far_relay:=$([ "${FAR_RELAY:-1}" = "1" ] && echo true || echo false) far_relay_m:=$(as_float "${FAR_RELAY_M:-4.0}") far_relay_min_goal:=$(as_float "${FAR_RELAY_MIN:-1.5}")" \
  > "$OUT/launch.log" 2>&1 < /dev/null 9>&- &
sleep 3; PGID=$(cat "$OUT/pgid" 2>/dev/null || echo "")
say "已启动：PGID=$PGID  日志 $OUT/launch.log"
rec "\"check\":\"launch\",\"pgid\":\"$PGID\""

if [ "${SMOKE:-0}" = "1" ]; then
  say "SMOKE 模式：等就绪后做 10 项自检（不跑探索）"
  # 等"栈就绪 + 运行期补丁全部落地"两个条件都满足，再做自检
  for i in $(seq 1 30); do
    grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null && break
    sleep 3
  done
  for i in $(seq 1 25); do
    grep -q "全部阶段完成" "$OUT/launch.log" 2>/dev/null && break
    sleep 2
  done
  # 保守口径：再等最多 50 s 等"第一个探索目标派发"，用它证明探索环真的转起来了
  for i in $(seq 1 25); do
    grep -q "navigation goal dispatched" "$OUT/launch.log" 2>/dev/null && break
    sleep 2
  done
  sleep 3
  bash harness/smoke_check.sh "$OUT/launch.log" | tee -a "$OUT/harness.log"
  SMOKE_RC=${PIPESTATUS[0]}
  rec "\"check\":\"smoke\",\"rc\":$SMOKE_RC"
  [ "$SMOKE_RC" = "0" ] && rec '"verdict":"PASS_smoke"' || rec '"verdict":"FAIL_smoke"'
  PGID_SAVED=$PGID
  teardown_once() {
    [ -n "$PGID_SAVED" ] && kill -TERM -- "-$PGID_SAVED" 2>/dev/null
    for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
    pkill -x gzserver 2>/dev/null; sleep 2
  }
  teardown_once
  echo "JSONL: $J"
  exit "$SMOKE_RC"
fi

if [ "$HOLD" -gt 0 ]; then
  sleep "$HOLD"
  say "空跑 $HOLD s 结束，执行 teardown 验证"
else
  # 全流程模式：等会话自然收尾（会话结束 / 地图落盘 / 超时上限）
  # 余量按**真实时间**计：此前用 `waited=$((waited+10))` 累加，但每拍实际要 16 s
  # （sleep 10 + mem_sample 里的 ps/ros2），"480 s" 实际跑了 770 s —— 一轮 14 分钟，
  # 5 轮 ×2 批 = 2.4 h，严重拖慢迭代。改为按 date 差值计时，余量由 ROUND_MARGIN 控制。
  MARGIN=${ROUND_MARGIN:-300}
  MAXW=$(python3 -c "print(int(float('$EXPLORE') + float('$MARGIN')))")
  say "全流程模式：最多等待 ${MAXW}s（探索 ${EXPLORE}s + 返航/存图余量 ${MARGIN}s，按真实时间计）"
  T0=$(date +%s); waited=0
  AUDIT_DONE=0
  BACKSTEP_EXPECT=$([ "${EXPLORE_BACKSTEP:-1}" = "1" ] && echo 1 || echo 0)
  while [ "$waited" -lt "$MAXW" ]; do
    sleep 10; waited=$(( $(date +%s) - T0 )); mem_sample
    # P0-乙：起栈就绪后立刻做参数落地自检（此时 patch_sim_time 已跑完、栈还活着）
    if [ "$AUDIT_DONE" = "0" ] && grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null \
       && grep -q "全部阶段完成" "$OUT/launch.log" 2>/dev/null; then
      bash harness/param_audit.sh "$OUT/param_audit.json" "$BACKSTEP_EXPECT" "${RRT_WAIT:-0}" "${FRONT_STOP:-0}" 2>&1 | tee -a "$OUT/harness.log"
      if [ "${PIPESTATUS[0]}" = "0" ]; then rec '"check":"param_audit","ok":true'
      else say "✗ 参数自检未全通过 → 本轮 invalid(param_audit)"; rec '"check":"param_audit","ok":false'; fi
      AUDIT_DONE=1
    fi
    if grep -q "会话结束" "$OUT/launch.log" 2>/dev/null; then say "检测到会话结束（${waited}s）"; break; fi
    # 注意：会话驱动在返航期每 2 s 滚动存图，"地图文件出现"不再是收尾信号，
    # 必须等"会话结束"或 manager 明确返航到位（否则会在返航刚开始时就收尾）。
    # 只认 manager 的正向完成行；不要匹配"会话未走到返航到位"这类否定句
    if grep -qE "return-home completed by position|return-home goal reached" "$OUT/launch.log" 2>/dev/null; then
      say "返航到位（${waited}s）"; sleep 8; break
    fi
    pgrep -x gzserver >/dev/null || { say "gzserver 已退出（${waited}s）"; break; }
  done
fi

# ---- 3) teardown（按 PGID） ----
teardown() {
  [ -n "$PGID" ] && kill -TERM -- "-$PGID" 2>/dev/null
  for i in $(seq 1 20); do pgrep -x gzserver >/dev/null || break; sleep 1; done
  pkill -x gzserver 2>/dev/null
  for i in $(seq 1 10); do [ "$(timeout 8 ros2 node list 2>/dev/null | wc -l)" -le 1 ] && break; sleep 1; done
}
teardown
NODES=$(timeout 10 ros2 node list 2>/dev/null | wc -l); GZ=$(pgrep -cx gzserver)
RES=$(pgrep -af "cartographer_node|sim_session_driver|controller_server|planner_server|bt_navigator|imu_bias_shim|scan_deadzone_relay" 2>/dev/null | grep -E "/(lib/|install/|opt/ros)" | grep -v "harness/round.sh" | awk '{print $1}' | tr '\n' ' ')
say "收尾后：ROS 节点数=$NODES  gzserver=$GZ  残留=[${RES}]"
rec "\"check\":\"post_teardown\",\"nodes\":$NODES,\"gzserver\":$GZ,\"residue\":\"${RES}\""
[ "$GZ" = "0" ] && [ -z "$RES" ] && rec '"verdict":"PASS_clean"' || rec '"verdict":"FAIL_residue"'

# ---- 起栈诊断：ready_guard 时序 + 生命周期 nudge 事件 ----
if grep -q "ready_guard_timing" "$OUT/launch.log" 2>/dev/null; then
  say "起栈时序：$(grep -o 'ready_guard_timing[^\n]*' "$OUT/launch.log" | tail -1 | cut -c1-110)"
elif grep -qE "Nav2 did not become active|did not become ready" "$OUT/launch.log" 2>/dev/null; then
  say "✗ 起栈失败：nav2 生命周期未在 guard 超时内激活"
  rec '"check":"startup_fail"'
fi
if [ -s "$OUT/nudge.jsonl" ]; then
  python3 - "$OUT/nudge.jsonl" <<'PYEOF3'
import json, sys
rows = []
for line in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    line = line.strip()
    if line:
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
nudges = [r for r in rows if r.get('type') == 'nudge']
summ = [r for r in rows if r.get('type') == 'summary']
print('[nudge] 插队 %d 次%s' % (len(nudges),
      ('；' + ', '.join('%s×%d' % (n['node'].lstrip('/'), n['attempt']) for n in nudges)) if nudges else '（起栈正常，未插队）'))
if summ:
    print('  收尾：%d 个 ACTIVE' % len(summ[-1].get('active', [])))
PYEOF3
fi

# ---- P1-2：静止看门狗事件汇总 ----
if [ -f "$OUT/watchdog.jsonl" ]; then
  python3 - "$OUT/watchdog.jsonl" <<'PYEOF2'
import json, sys, collections
rows = []
for line in open(sys.argv[1], encoding='utf-8', errors='ignore'):
    line = line.strip()
    if line:
        try:
            rows.append(json.loads(line))
        except Exception:
            pass
why = collections.Counter(r.get('why', '?') for r in rows)
print('[watchdog] 静止事件 %d 次：%s' % (len(rows), dict(why)))
for r in rows[:6]:
    print('    %.1fs @(%.2f,%.2f) %s err=%s'
          % (r.get('stall_sec', 0), r['pos'][0], r['pos'][1], r.get('why'), r.get('last_error')))
PYEOF2
fi

# ---- 方案A：撤销视觉图层注入 + 内存峰值汇总 ----
python3 harness/vision_nav_params.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
python3 harness/patch_car_launch.py --restore 2>&1 | tee -a "$OUT/harness.log" || true
if [ -s "$MEMLOG" ]; then
  python3 - "$MEMLOG" "$J" <<'PYEOF'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
# 只在"栈还活着"的采样区间里算 CPU（收尾后进程消失，tick 会归零，否则会算出负值）
rows_live = [r for r in rows if (r.get('gz_mb') or 0) > 0 and (r.get('cpu_gz') or 0) >= 0]
peak = {k: max((r.get(k) or 0) for r in rows) for k in
        ('gz_mb', 'vision_mb', 'nav2_mb', 'carto_mb')}
low = min((r.get('avail_mb') or 0) for r in rows)
summary = {'samples': len(rows), 'peak_gz_mb': peak['gz_mb'], 'peak_vision_mb': peak['vision_mb'],
           'peak_nav2_mb': peak['nav2_mb'], 'peak_carto_mb': peak['carto_mb'],
           'min_avail_mb': low}
# CPU 换算：100 = 一个核跑满；tick 差 / (时间差 × CLK_TCK)
try:
    import os as _os
    hz = _os.sysconf('SC_CLK_TCK')
    src = rows_live if len(rows_live) >= 2 else rows
    dt = max(1, src[-1]['t'] - src[0]['t'])
    for k, tag in (('cpu_gz', 'gz'), ('cpu_vision', 'vision'),
                   ('cpu_nav2', 'nav2'), ('cpu_carto', 'carto')):
        vals = [r.get(k) for r in src if r.get(k) is not None]
        if len(vals) >= 2:
            summary['cpu_%s_pct' % tag] = round(100.0 * (vals[-1] - vals[0]) / (dt * hz), 1)
except Exception:
    pass
with open(sys.argv[2], 'a') as f:
    f.write('{"check":"memory",' + json.dumps(summary, ensure_ascii=False)[1:] + '\n')
print('[memory] ' + json.dumps(summary, ensure_ascii=False))
PYEOF
fi

# ---- 4) 撞障取证汇总（P0）----
if [ -f "$OUT/probe.jsonl" ]; then
  python3 - "$OUT/probe.jsonl" "$J" <<'PYEOF'
import json, sys, collections
src, jout = sys.argv[1], sys.argv[2]
c = collections.Counter(); mind = None; rev = []
for line in open(src, encoding='utf-8', errors='ignore'):
    line = line.strip()
    if not line:
        continue
    try:
        r = json.loads(line)
    except Exception:
        continue
    t = r.get('type')
    if t in ('contact', 'graze', 'alert', 'stall', 'giveup', 'no_direction'):
        c[t] += 1
    if t == 'reverse':
        if r.get('phase') == 'end':
            rev.append(r)
    for k in ('min_d', 'min_d_now', 'min_d_ever'):
        v = r.get(k)
        if isinstance(v, (int, float)) and v >= 0 and (mind is None or v < mind):
            mind = v
last = {}
for line in open(src, encoding='utf-8', errors='ignore'):
    try:
        r = json.loads(line)
    except Exception:
        continue
    if r.get('type') == 'summary':
        last = r
hard = [x for x in rev if (x.get('dist') or 0.0) >= 0.05 and not x.get('assist')]
assist = [x for x in rev if x.get('assist')]
summary = {'contacts': c['contact'], 'grazes': c['graze'], 'alerts': c['alert'],
           'stalls': c['stall'], 'reverse_windows': len(rev),
           'reverse_m': round(sum(x.get('dist', 0.0) for x in rev), 3),
           'reverse_assist_windows': len(assist),
           'reverse_assist_m': round(sum((x.get('dist') or 0.0) for x in assist), 3),
           'reverse_hard_windows': len(hard),
           'reverse_hard_m': round(sum((x.get('dist') or 0.0) for x in hard), 3),
           'chassis_contacts': last.get('chassis_contacts', 0),
           'wheel_contacts': last.get('wheel_contacts', 0),
           'physical_contacts': last.get('chassis_contacts', 0) + last.get('wheel_contacts', 0),
           'min_clearance_ever': round(mind, 3) if mind is not None else None}
line = '{"tag":"%s","check":"collision","%s"}' % (jout.rsplit('/', 1)[-1], '')
with open(jout, 'a') as f:
    f.write('{"check":"collision","summary":%s}\n' % json.dumps(summary, ensure_ascii=False))
print('[collision] ' + json.dumps(summary, ensure_ascii=False))
PYEOF
fi
# 地图快照（下一轮开头会删 explore_latest.*，不快照就再也评不到本轮的地图了）
MF=/home/kbf/stemm_sim_ws/maps/explore_latest.pgm
if [ -f "$MF" ] && [ "$(stat -c %Y "$MF")" -ge "$T_START" ]; then
  cp -f "$MF" "$OUT/map.pgm"
  cp -f "/home/kbf/stemm_sim_ws/maps/explore_latest.yaml" "$OUT/map.yaml" 2>/dev/null
  say "地图快照：$OUT/map.pgm（$(stat -c %s "$MF") B）"
fi
# 本轮验收（七项 + P0 两项）
if [ -f "$OUT/launch.log" ]; then
  timeout 180 python3 harness/evaluate.py "$OUT/launch.log" "$TAG" "$OUT/probe.jsonl" --p0 \
    > "$OUT/evaluate.json" 2>&1 || true
  # 候选打分专用指标（独立于 PASS 验收）：供给/收获/打分质量/守门
  timeout 120 python3 harness/candidate_metrics.py "$OUT" --json "$OUT/cand_metrics.json" \
    > "$OUT/cand_metrics.md" 2>&1 || true
  python3 - "$OUT/evaluate.json" <<'PYEOF'
import json,sys
try:
    d=json.load(open(sys.argv[1]))
    print('[evaluate] verdict=%s checks=%s' % (d['verdict'],
          {k:v for k,v in d['checks'].items() if not v} or 'all true'))
except Exception as e:
    print('[evaluate] 解析失败:', e)
PYEOF
fi
if [ -f "$OUT/launch.log" ]; then
  echo
  # 节点猝死体检（2026-10-26 新增）：提交 aeef06c 让 escape_supervisor 在探索期第一次
  # tick 就 Traceback 退出；launch 不为这类可选节点设硬失败，整轮只是"少一个节点"，
  # digest 照样判 PASS —— 34+ 轮 A/B 因此被污染（详见 docs/P9 §14）。这里把它变成
  # 显性产物（dead_nodes.txt）+ 显著告警，后续再并入 digest 的失败项。
  # 只认"真崩"：Traceback 块的异常行不在关停白名单里。
  # （收关时 rclpy 每个节点都会打 ExternalShutdownException + exit code 1，那是正常噪声，
  #  第一版体检把它当真崩 → 每轮都被判无效，批次白重跑。见 harness/check_node_health.py）
  if timeout 60 python3 harness/check_node_health.py "$OUT/launch.log" > "$OUT/dead_nodes.txt" 2>&1; then
    :
  else
    say "⚠ 本轮有节点真崩，见 $OUT/dead_nodes.txt —— 该轮数据不可作为结论"
  fi
  timeout 180 python3 harness/report.py "$OUT" "$TAG" > "$OUT/report.stdout" 2>&1 || true
  timeout 180 python3 harness/round_digest.py "$OUT" > "$OUT/digest.stdout" 2>&1 || true
  head -40 "$OUT/report.stdout"
  echo "[report] 完整报告：$OUT/report.md"
fi
if [ -f "$OUT/supervisor.jsonl" ]; then
  echo "[supervisor] 事件汇总：" 
  grep -o '"type": "[a-z_0-9]*"' "$OUT/supervisor.jsonl" | sort | uniq -c || true
fi
echo "JSONL: $J"
