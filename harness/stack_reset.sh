#!/usr/bin/env bash
# stack_reset.sh —— 唯一被允许的"整栈清理"入口（顺序不可颠倒）。
#
# 2026-10-28 事故的直接教训：
#   * 只 `pkill -x gzserver` 不等于清场：RSP / nav2 / amcl / vision / spawn_entity
#     全都还活着，紧接着 `rm -f /dev/shm/fastrtps_*` 会把这些**活进程**的
#     Fast-DDS 段删掉（196 个进程映射悬空）→ domain 42 发现机制整体失效。
#   * `pkill -f <宽模式>` 会自伤：模式若出现在自己命令行里（本会话踩了 4 次），
#     连自己一起杀。所以这里一律 pgrep 枚举 PID 再按 PID/进程组杀。
#   * 后台实验脚本若用 `setsid nohup` 脱离生命周期，工具调用被中断后它还在跑，
#     两个实例各 `pkill gzserver` 互杀 → 谁的栈都活不过 3 分钟。
#     ⇒ 清理时必须先按 pidfile / 进程组收掉这些"脱离的脚本"，再动栈。
#
# 用法：
#   bash harness/stack_reset.sh                    # 全量清理 + 复检
#   bash harness/stack_reset.sh /tmp/run_a.pid     # 顺便收掉这些 pid 文件对应的进程组
#
# 环境：不 source 任何 setup.bash（清理不应依赖 ROS 环境）。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
PIDFILES=("$@")

echo "[reset] ① 收掉脱离生命周期的实验脚本（pidfile → 整个进程组）"
for f in "${PIDFILES[@]:-}"; do
  [ -n "${f:-}" ] && [ -f "$f" ] || continue
  pid=$(head -1 "$f" | tr -dc '0-9')
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    echo "[reset]   $f pid=$pid pgid=${pgid:-$pid}"
    kill -TERM -- "-${pgid:-$pid}" 2>/dev/null
    sleep 2
    kill -KILL -- "-${pgid:-$pid}" 2>/dev/null
  fi
done

echo "[reset] ② 整栈按可执行路径枚举清理"
bash harness/stack_kill.sh kill 2>&1 | tail -2

echo "[reset] ③ 收掉 stack_kill 覆盖不到的零散节点（按 PID，不用宽 pkill）"
PATTERNS='ros2 topic (hz|echo)|ros2 param |ros2 node |ros2 service |gazebo_ros/spawn_entity|ros2 daemon'
STRAY=$(ps -eo pid,cmd | grep -E "$PATTERNS" | grep -v grep | awk '{print $1}' | grep -vw "$$" | grep -vw "$PPID")
if [ -n "$STRAY" ]; then
  echo "[reset]   零散进程 $(echo "$STRAY" | wc -w) 个"
  for p in $STRAY; do kill -9 "$p" 2>/dev/null; done
fi

echo "[reset] ④ 等零参与者（最多 30 s）"
for _ in $(seq 1 30); do
  gz=$(pgrep -cx gzserver 2>/dev/null || echo 0)
  ros=$(ps -eo cmd | grep -E 'opt/ros/humble|stemm_(sim|explore|exploration)_ws/install' | grep -vc grep || true)
  [ "$gz" = "0" ] && [ "${ros:-1}" = "0" ] && break
  sleep 1
done
echo "[reset]   gzserver=$gz ROS进程=${ros:-?}"

echo "[reset] ⑤ 停 ros2 daemon（LOCALHOST_ONLY 下起过的 daemon 会污染 graph 视图）"
bash -c 'source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1; source /opt/ros/humble/setup.bash >/dev/null 2>&1; export ROS_LOCALHOST_ONLY=1; timeout 15 ros2 daemon stop >/dev/null 2>&1' || true
DAEMON=$(ps -eo pid,cmd | grep 'ros2cli[.]daemon[.]daemonize' | grep -v grep | awk '{print $1}')
[ -n "$DAEMON" ] && for p in $DAEMON; do kill -9 "$p" 2>/dev/null; done
sleep 1

echo "[reset] ⑥ 断言零参与者之后才清 fastrtps 段（顺序错了就是在删活人的内存）"
HOLD=0
for p in /proc/[0-9]*/maps; do grep -q 'fastrtps.*(deleted)' "$p" 2>/dev/null && HOLD=$((HOLD + 1)); done
if [ "$HOLD" -gt 0 ]; then
  echo "[reset] ✗ 仍有 $HOLD 个进程映射着已删的 fastrtps 段——先杀进程，不要删 shm"
  exit 1
fi
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
rm -f /tmp/stemm_round.lock 2>/dev/null

echo "[reset] ⑦ 还原注入的参数/补丁（可回退纪律）"
for s in nav_params.py vision_nav_params.py patch_car_launch.py; do
  [ -f "harness/$s" ] && python3 "harness/$s" --restore >/dev/null 2>&1
done
echo "[reset]   已还原"

echo "[reset] ⑧ 复检"
sleep 1
bash harness/preflight_dds.sh || exit 1
