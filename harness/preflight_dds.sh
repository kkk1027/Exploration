#!/usr/bin/env bash
# preflight_dds.sh —— 起栈前的 DDS/环境健康闸门。
#
# 为什么需要它（2026-10-28 事故）：
#   临时实验脚本只 `pkill -x gzserver` 就 `rm -f /dev/shm/fastrtps_*`，
#   把**还在跑的**节点的 Fast-DDS 共享内存段删掉 → 196 个进程映射悬空，
#   domain 42 的发现机制整体死掉：`ros2 node list` 返回 0，
#   `/robot_description` 变成 Unknown topic，于是
#   spawn_entity 永久卡在 "Waiting for entity xml"、lifecycle_manager 永久等
#   map_server/get_state，而界面上只看到"无相机数据 → 起栈失败重试"。
#   ⇒ 起栈前必须先问一句"这个 domain 现在还干净吗"，而不是盲目重试。
#
# 用法：
#   bash harness/preflight_dds.sh      # 人类可读报告（会自清理"无主"fastrtps 残留）
#   bash harness/preflight_dds.sh -q   # 只输出一行 OK / FAIL:...（给脚本用）
#   退出码：0 = 可以起栈，1 = 拒绝起栈（原因已打印）
#
# 判据（任一不满足即 FAIL）：
#   gzserver 计数 0            —— 上一轮仿真没死透，会抢 Gazebo master
#   ROS 节点进程 0             —— 上一轮 launch 的遗孤
#   悬空 fastrtps 段持有者 0   —— 有进程映射着已 unlink 的段 ⇒ 通信已残
#   ros2 node list < 3         —— 干净 domain 里新 shell 应几乎看不到节点
#   load1 < 2*nproc            —— 超载时握手/生命周期服务全超时，跑出的数不能用
# 无主 fastrtps 段（没有任何进程映射）不算 FAIL：登记后直接删，保证闸门可幂等重跑。
set -u
QUIET=0
[ "${1:-}" = "-q" ] && QUIET=1
say() { [ "$QUIET" = "1" ] || echo "$@"; }

NPROC=$(nproc)
# ★★ 2026-10-28 域隔离（P0 误伤修复）：本机还有**域 0 的真车栈**在跑，
# 之前把它们算进"ROS 遗孤/悬挂映射"，并删掉它们映射的 /dev/shm 段。
# 这里所有判定只看仿真域（ROS_DOMAIN_ID=42），删 shm 也要求"全局零 ROS 进程"。
SIM_DOMAIN_EXPECT="${SIM_DOMAIN_EXPECT:-42}"
is_ours() { tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=${SIM_DOMAIN_EXPECT}"; }
count_ours() {   # count_ours <pgrep 模式...>  → 只数本域
  local n=0 p
  for p in $(pgrep -f "$1" 2>/dev/null); do is_ours "$p" && n=$((n+1)); done
  echo "$n"
}
FAIL=0
WHY=()

# 悬空段持有者计数：某进程 maps 里含 "fastrtps...(deleted)" ⇒ 它的 DDS 段被删掉了
dangling_holders() {   # 只数**本仿真域**里映射着已删段的进程
  local n=0 pid
  for p in /proc/[0-9]*/maps; do
    grep -q 'fastrtps.*(deleted)' "$p" 2>/dev/null || continue
    pid=$(basename "$(dirname "$p")")
    is_ours "$pid" && n=$((n + 1))
  done
  echo "$n"
}
other_domain_ros() {   # 其它域（含域0真车）的 ROS 进程数
  local n=0 pid
  for pid in $(pgrep -f '/opt/ros/humble|ros2 launch' 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    is_ours "$pid" || n=$((n + 1))
  done
  echo "$n"
}
# 只在"零持有者"时清无主残留段（顺序错了就是在删活人的内存）
clean_orphan_shm() {
  local hold n others
  hold=$(dangling_holders)
  if [ "$hold" -gt 0 ]; then
    echo "dangling=$hold"
    return 1
  fi
  others=$(other_domain_ros)
  if [ "${others:-0}" -gt 0 ]; then
    # 别的域在跑 ⇒ 它们也可能映射 /dev/shm/fastrtps_*，删了就是误伤（已踩过）
    n=$(ls /dev/shm 2>/dev/null | grep -cE '^fastrtps_|^sem\.fastrtps' || true)
    echo "skip(other-domain=${others},shm=${n:-0})"
    return 0
  fi
  n=$(ls /dev/shm 2>/dev/null | grep -cE '^fastrtps_|^sem\.fastrtps' || true)
  if [ "${n:-0}" -gt 0 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    echo "cleaned=$n"
  else
    echo "clean"
  fi
  return 0
}

# ── 0) ros2 daemon 先停：它自己就是一个 DDS 参与者，会持有被删段的映射，
#       也会用陈旧缓存污染 graph 视图（LOCALHOST_ONLY 下起过的 daemon 尤其）。
DAEMON=$(ps -eo pid,cmd | grep 'ros2cli[.]daemon' | grep -v grep | awk '{print $1}')
if [ -n "$DAEMON" ]; then
  for p in $DAEMON; do kill -9 "$p" 2>/dev/null; done
  sleep 1
  say "[preflight] 已停 ros2 daemon（$(echo "$DAEMON" | wc -w) 个）"
fi

# ── 1) 进程层 ────────────────────────────────────────────────
GZ=$(pgrep -cx gzserver 2>/dev/null || true); GZ=${GZ:-0}
[ "$GZ" -gt 0 ] && { FAIL=1; WHY+=("gzserver=$GZ（>0：上一轮仿真没死透，会抢 Gazebo master）"); }

ROS=0
for pid in $(pgrep -f 'stemm_(sim|explore|exploration)_ws/install|/opt/ros/humble/lib/nav2_|ros2 launch stemm' 2>/dev/null); do
  is_ours "$pid" && ROS=$((ROS + 1))
done
[ "${ROS:-0}" -gt 0 ] && { FAIL=1; WHY+=("ROS 进程=$ROS（>0：上一轮 launch 的遗孤）"); }

HOLD=$(dangling_holders)
[ "$HOLD" -gt 0 ] && { FAIL=1; WHY+=("悬挂映射=$HOLD 个进程（fastrtps 段被删但进程还映射着 ⇒ DDS 已残，必须整栈重启）"); }

# ── 2) 无主残留（只登记 + 安全清理，不算失败）───────────────
if ! SHM_NOTE=$(clean_orphan_shm); then
  FAIL=1
  WHY+=("清 shm 前发现 $SHM_NOTE 个持有者，拒绝删除")
fi
say "[preflight] /dev/shm 无主 fastrtps 残留：${SHM_NOTE:-clean}"

# ── 3) 发现能力（干净 domain 里应当几乎看不到节点）──────────
NODES=$(bash -c 'source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
                 source /opt/ros/humble/setup.bash >/dev/null 2>&1
                 export ROS_LOCALHOST_ONLY=1
                 timeout 12 ros2 node list --no-daemon 2>/dev/null | grep -c .' || true)
NODES=${NODES:-0}
[ "$NODES" -ge 3 ] && { FAIL=1; WHY+=("node list=$NODES（≥3：还有别的栈在这个 domain 上跑）"); }
# 探针自身会留下几个段：探针已退出，按零持有者安全清掉，保证闸门可重跑
clean_orphan_shm >/dev/null 2>&1 || true

# ── 4) 负载 ─────────────────────────────────────────────────
LOAD1=$(cut -d' ' -f1 /proc/loadavg)
OVER=$(awk -v l="$LOAD1" -v n="$NPROC" 'BEGIN{print (l > 2*n) ? 1 : 0}')
[ "$OVER" = "1" ] && { FAIL=1; WHY+=("load1=$LOAD1 超过 2×nproc=$((2 * NPROC))（超载会让 DDS 握手与生命周期服务全超时）"); }

# ── 结论 ────────────────────────────────────────────────────
if [ "$FAIL" = "0" ]; then
  if [ "$QUIET" = "1" ]; then
    echo "OK"
  else
    echo "[preflight] ✓ 环境干净：gz=0 ros=0 悬挂=0 nodes=$NODES load1=$LOAD1 (nproc=$NPROC)"
    echo "[preflight] ✓ 可以起栈"
  fi
  exit 0
fi
if [ "$QUIET" = "1" ]; then
  echo "FAIL: ${WHY[*]}"
else
  echo "[preflight] ✗ 拒绝起栈，原因："
  for w in "${WHY[@]}"; do echo "           - $w"; done
  echo "[preflight] 修法：bash harness/stack_reset.sh（先停脚本 → 整栈杀 → 等零参与 → 清 shm → 停 daemon → 复检）"
fi
exit 1
