#!/usr/bin/env bash
# param_audit.sh —— P0-乙：运行期参数是否真的落地（防"看起来正常但没设上"）
# 用法: param_audit.sh <输出json> [expect_backstep 1|0] [rrt_wait] [front_stop]
#
# 2026-10-26 重写：原来靠 `ros2 param dump`，而本机（手机热点 +
# ROS_LOCALHOST_ONLY=1）下 ros2cli daemon 发现不到节点 → dump 恒空 →
# 自检恒 0/8、每轮假失败 invalid(param_audit)，还要白等 4.5 分钟重试。
# 现在交给 harness/param_audit_probe.py：rclpy 直连 <node>/get_parameters。
set +u
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy
if [ "${ROS_LOCALHOST_ONLY:-}" != "1" ]; then unset ROS_LOCALHOST_ONLY; fi
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
H=/home/kbf/stemm_exploration_sim_ws/harness
timeout 60 python3 "$H/param_audit_probe.py" --out "${1:-/dev/null}" --backstep "${2:-1}" \
  --rrt-wait "${3:-0}" --front-stop "${4:-0}"
RC=$?
[ "$RC" = "124" ] && echo "  [audit] ✗ 自检超时（60 s）"
exit "$RC"
