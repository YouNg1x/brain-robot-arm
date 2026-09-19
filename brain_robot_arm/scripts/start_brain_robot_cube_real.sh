#!/usr/bin/env bash
# Protected real-arm purple-cube visual search.
set -Eeuo pipefail

ROS_SETUP=/opt/ros/humble/setup.bash
PIPER_WS="${PIPER_ROS_WS:-$HOME/piper_ros}"
APP_WS="${BRAIN_ROBOT_WS:-$HOME/piper_ws}"
CAMERA_WS="${BRAIN_ROBOT_CAMERA_WS:-$HOME/ros2_ws}"
INIT_SCRIPT="${PIPER_INIT_SCRIPT:-$HOME/init_arm.sh}"
WAIT_SECONDS="${BRAIN_ROBOT_WAIT_SECONDS:-45}"
PIDS=()

fail() { echo "[错误] $*" >&2; exit 1; }
cleanup() {
  trap - EXIT INT TERM
  echo; echo "[关闭] 先关闭实体运动门..."
  timeout 5s ros2 service call /visual_search_controller/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /piper_jog_adapter/disarm std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /piper_jog_adapter/disable_motion std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  echo "[关闭] 停止本脚本启动的视觉、相机和驱动进程..."
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
  echo "[完成] 一键流程已停止；运动门已关闭。"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ -n "${CONDA_PREFIX:-}" ]] && command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)"
  while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
fi
[[ -f "$ROS_SETUP" && -f "$PIPER_WS/install/setup.bash" && -f "$APP_WS/install/setup.bash" ]] ||
  fail "ROS2、PiPER 或应用工作空间尚未就绪。"
[[ -f "$INIT_SCRIPT" ]] || fail "找不到实体初始化脚本：$INIT_SCRIPT"

set +u
source "$ROS_SETUP"
source "$PIPER_WS/install/setup.bash"
source "$APP_WS/install/setup.bash"
if [[ -f "$CAMERA_WS/install/setup.bash" ]]; then
  source "$CAMERA_WS/install/setup.bash"
fi
set -u

wait_for() {
  local kind="$1" name="$2" deadline=$((SECONDS + WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    timeout 5s ros2 "$kind" list 2>/dev/null | grep -Fxq "$name" && return 0
    sleep 1
  done
  fail "等待 $name 超时。"
}

start_group() { setsid "$@" & PIDS+=("$!"); }

node_exists() {
  timeout 5s ros2 node list 2>/dev/null | grep -Fxq "$1"
}

topic_exists() {
  timeout 5s ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

echo "=================================================="
echo " PiPER 实体紫色方块视觉抓取（一键保护模式）"
echo " 自动启动 CAN、PiPER 驱动、相机、MoveIt、Servo 和检测器"
echo "=================================================="
echo "[1/7] 检查并初始化实体 CAN/机械臂..."
if ! node_exists /piper_ctrl_single_node; then
  bash "$INIT_SCRIPT" || fail "实体机械臂初始化失败。"
  start_group ros2 run piper piper_single_ctrl \
    --ros-args \
    -p can_port:=can0 \
    -p auto_enable:=false \
    -p gripper_exist:=true \
    -r joint_states_single:=/joint_states \
    -r joint_ctrl_single:=/joint_commands
else
  echo "[复用] 已检测到 /piper_ctrl_single_node"
fi
wait_for topic /joint_states

echo "[2/7] 检查并启动 RGB-D 相机..."
if ! node_exists /camera/camera; then
  start_group ros2 launch astra_camera astra.launch.py
else
  echo "[复用] 已检测到 /camera/camera"
fi
wait_for topic /camera/color/image_raw
wait_for topic /camera/depth/image_raw

echo "[3/7] 启动紫色方块 RGB-D 检测..."
start_group ros2 launch brain_robot_ball_pick cube_detector.launch.py
wait_for topic /brain_robot_vision/debug_image

echo "[4/7] 启动 MoveIt、Servo 和真机保护适配器..."
start_group ros2 launch brain_robot_ball_pick cube_visual_search.launch.py
wait_for node /move_group
wait_for node /servo_node
wait_for node /piper_jog_adapter
wait_for node /grasp_lift_executor
wait_for service /piper_jog_adapter/arm
wait_for service /grasp_lift_executor/execute

echo "[5/7] 检查完整 MoveIt 反馈..."
wait_for topic /piper_moveit_joint_states
echo "[6/7] 打开识别窗口..."
start_group ros2 run image_view image_view --ros-args -r image:=/camera/color/image_raw
start_group ros2 run image_view image_view --ros-args -r image:=/brain_robot_vision/debug_image
echo "[7/7] 全部组件已就绪。"
echo
echo "操作顺序：确认工作区安全后，按 1 开启运动门，按 2 使能适配器，按 3 开始搜寻。"
echo "目标进入 GRASP_READY 后，确认夹爪附近无障碍再按 6 执行抓取；按 4 停止，按 5 失能适配器，按 0 退出。"
while true; do
  if [[ -t 0 ]] && read -r -s -n 1 -t 1 key; then
    case "$key" in
      1) timeout 5s ros2 service call /piper_jog_adapter/enable_motion std_srvs/srv/Trigger '{}' || true ;;
      2) timeout 5s ros2 service call /piper_jog_adapter/arm std_srvs/srv/Trigger '{}' || true ;;
      3) timeout 8s ros2 service call /visual_search_controller/start std_srvs/srv/Trigger '{}' || true ;;
      4) timeout 5s ros2 service call /visual_search_controller/stop std_srvs/srv/Trigger '{}' || true ;;
      5) timeout 5s ros2 service call /piper_jog_adapter/disarm std_srvs/srv/Trigger '{}' || true ;;
      6) timeout 8s ros2 service call /grasp_lift_executor/execute std_srvs/srv/Trigger '{}' || true ;;
      0) exit 0 ;;
    esac
  fi
done
