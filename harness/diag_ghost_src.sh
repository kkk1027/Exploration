#!/usr/bin/env bash
# diag_ghost_src.sh <tag> <tf_stamp:true|false>
#
# "鬼影溯源"实验台：nav_only（初始位姿=地图原点）→ 静止采样（原始朝向）
#   → 原地转约 1 rad → 再采样。
# 每轮都跑 vision_src_probe（按 z 带/径向看视觉点从哪儿来）+ ghost_probe 的轻量版指标。
set -u
cd /home/kbf/stemm_exploration_sim_ws || exit 9
TAG="${1:-src}"
TSV="${2:-true}"
OUT="logs/ghostsrc_$(date +%Y%m%d-%H%M%S)_${TAG}_stamp${TSV}"
mkdir -p "$OUT"

set +u
source /home/kbf/stemm_sim_ws/domain_guard.sh >/dev/null 2>&1
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_sim_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_explore_ws/install/setup.bash >/dev/null 2>&1
source /home/kbf/stemm_exploration_sim_ws/install/setup.bash >/dev/null 2>&1
export ROS_LOCALHOST_ONLY=1
set -u

exec 9>/tmp/stemm_round.lock
flock -n 9 || { echo "[src] 已有轮次在跑"; exit 3; }
bash harness/preflight_dds.sh -q || { echo "[src] 环境不健康"; exit 4; }

cleanup() {
  [ -n "${LPID:-}" ] && kill -TERM -- "-$LPID" 2>/dev/null
  sleep 3
  [ -n "${LPID:-}" ] && kill -KILL -- "-$LPID" 2>/dev/null
  bash harness/stack_kill.sh kill >/dev/null 2>&1
  python3 harness/vision_nav_params.py --restore >/dev/null 2>&1
  echo "[src] 收尾 gz=$(pgrep -cx gzserver 2>/dev/null || true)"
}
trap cleanup EXIT INT TERM

setsid ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py \
  map_yaml:=/home/kbf/stemm_sim_ws/maps/explore_latest.yaml \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0 \
  vision:=true vision_mode:=accum vision_tf_stamp:=$TSV \
  gui:=false rviz:=false vision_dump:="$OUT/vision_grid" > "$OUT/launch.log" 2>&1 &
LPID=$!
for _ in $(seq 1 90); do
  st=$(timeout 8 ros2 lifecycle get /amcl 2>/dev/null | head -1)
  case "$st" in active*) break ;; esac
  sleep 2
done
for _ in $(seq 1 60); do
  timeout 6 ros2 topic hz /vision/live_cloud 2>/dev/null | grep -q 'average rate' && break
  sleep 2
done
sleep 8

echo "[src] ===== 旋转前（原始朝向，车静止）=====" | tee "$OUT/src.log"
timeout 120 python3 harness/vision_src_probe.py --epochs 2 --sec 8 2>&1 | tee -a "$OUT/src.log"

echo "[src] ===== 原地转约 1 rad =====" | tee -a "$OUT/src.log"
# ★ 必须 9>&-（不继承轮次锁）并且 setsid（自成进程组），否则杀不干净：
#   上一轮就是这样把 flock 留给了一个还在 10 Hz 发 /cmd_vel 的孤儿。
setsid bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1
                export ROS_LOCALHOST_ONLY=1
                exec timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
                  "{angular: {z: 1.2}}"' >/dev/null 2>&1 9>&- &
PUBPID=$!
sleep 7
kill -TERM -- "-$PUBPID" 2>/dev/null; kill -9 $PUBPID 2>/dev/null
setsid bash -c 'source /opt/ros/humble/setup.bash >/dev/null 2>&1
                export ROS_LOCALHOST_ONLY=1
                ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"' >/dev/null 2>&1 9>&- &
sleep 5

echo "[src] ===== 旋转后（车静止）=====" | tee -a "$OUT/src.log"
timeout 120 python3 harness/vision_src_probe.py --epochs 2 --sec 8 2>&1 | tee -a "$OUT/src.log"

echo "[src] 输出 $OUT"
