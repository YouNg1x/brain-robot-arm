#!/usr/bin/env bash
# Protected real-arm purple-cube visual search.
set -Eeuo pipefail

ROS_SETUP=/opt/ros/humble/setup.bash
PIPER_WS="${PIPER_ROS_WS:-$HOME/piper_ros}"
APP_WS="${BRAIN_ROBOT_WS:-$HOME/piper_ws}"
CAMERA_WS="${BRAIN_ROBOT_CAMERA_WS:-$HOME/ros2_ws}"
LOG_DIR="${BRAIN_ROBOT_LOG_DIR:-$HOME/brain_robot_logs}"
LOG_MAX_MB="${BRAIN_ROBOT_LOG_MAX_MB:-20}"
WAIT_SECONDS="${BRAIN_ROBOT_WAIT_SECONDS:-45}"
PIDS=()
LOG_GUARD_PID=""
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

fail() { echo "[错误] $*" >&2; exit 1; }
cleanup() {
  trap - EXIT INT TERM
  echo; echo "[关闭] 先关闭实体运动门..."
  timeout 5s ros2 service call /visual_search_controller/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /piper_jog_adapter/disarm std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /piper_jog_adapter/disable_motion std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 8s ros2 service call /enable_srv piper_msgs/srv/Enable \
    '{enable_request: false}' >/dev/null 2>&1 || true
  echo "[关闭] 停止本脚本启动的视觉、相机和驱动进程..."
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  if [[ -n "$LOG_GUARD_PID" ]]; then
    kill "$LOG_GUARD_PID" 2>/dev/null || true
  fi
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

prepare_logs() {
  mkdir -p "$LOG_DIR"
  export ROS_LOG_DIR="$LOG_DIR/ros"
  mkdir -p "$ROS_LOG_DIR"
  # Keep startup logs useful without allowing repeated ROS output to consume
  # the VM disk.  The cleanup utility performs the same maintenance on demand.
  find "$LOG_DIR" -type f -name '*.log' -mtime +3 -delete 2>/dev/null || true
  find "$LOG_DIR" -type f \( -name '*.log' -o -name '*.log.*' \) \
    -size +"${LOG_MAX_MB}M" -exec truncate -s 0 {} \; 2>/dev/null || true
}

start_log_guard() {
  (
    while true; do
      find "$LOG_DIR" -type f \( -name '*.log' -o -name '*.log.*' \) \
        -size +"${LOG_MAX_MB}M" -exec truncate -s 0 {} \; 2>/dev/null || true
      # rsyslog can grow kern.log/syslog independently of ROS_LOG_DIR.  Only
      # truncate when sudo credentials are already cached by this startup.
      if sudo -n true 2>/dev/null; then
        sudo -n find /var/log -maxdepth 1 -type f \
          \( -name 'syslog*' -o -name 'kern.log*' \) \
          -size +200M -exec truncate -s 0 {} \; 2>/dev/null || true
      fi
      sleep 30
    done
  ) &
  LOG_GUARD_PID="$!"
}

start_group() {
  local tag="$1"
  shift
  local logfile="$LOG_DIR/${tag}.log"
  : > "$logfile"
  setsid "$@" >"$logfile" 2>&1 &
  PIDS+=("$!")
  echo "[后台] ${tag} 日志：${logfile}"
}

configure_can() {
  command -v ip >/dev/null 2>&1 || fail "未找到 ip 命令。"
  ip link show can0 >/dev/null 2>&1 || fail "未找到 can0；请确认 USB-CAN 已连接。"
  echo "[CAN] 重载 gs_usb 并配置 can0 为 1 Mbps..."
  sudo ip link set can0 down 2>/dev/null || true
  sudo modprobe -r gs_usb 2>/dev/null || true
  sudo modprobe gs_usb || fail "无法加载 gs_usb 驱动。"
  sleep 1
  sudo ip link set can0 down 2>/dev/null || true
  sudo ip link set can0 type can bitrate 1000000 2>/dev/null ||
    fail "无法配置 can0；请检查 USB-CAN 连接。"
  sudo ip link set can0 up || fail "无法启动 can0。"
  ip -details link show can0 | grep -E 'state (UP|UNKNOWN)|bitrate 1000000' >/dev/null ||
    fail "can0 未处于 1 Mbps 工作状态。"
}

node_exists() {
  timeout 5s ros2 node list 2>/dev/null | grep -Fxq "$1"
}

topic_exists() {
  timeout 5s ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

enable_physical_motion() {
  local result
  result=$(timeout 8s ros2 service call /enable_srv piper_msgs/srv/Enable \
    '{enable_request: true}' 2>&1) || result=""
  if ! grep -Eq 'enable_response=True' <<<"$result"; then
    echo "[错误] PiPER 实体使能失败，未启动复位。"
    return 1
  fi
  result=$(timeout 5s ros2 service call /piper_jog_adapter/enable_motion \
    std_srvs/srv/Trigger '{}' 2>&1) || result=""
  if ! grep -Eq 'success=True' <<<"$result"; then
    echo "[错误] 运动门开启失败。"
    return 1
  fi
  result=$(timeout 5s ros2 service call /piper_jog_adapter/arm \
    std_srvs/srv/Trigger '{}' 2>&1) || result=""
  if ! grep -Eq 'success=True' <<<"$result"; then
    echo "[错误] 适配器 arm 失败。"
    return 1
  fi
}

start_visual_sequence() {
  echo "[启动] 从零点移动到观测姿态..."
  local result
  result=$(timeout 8s ros2 service call /visual_search_controller/start \
    std_srvs/srv/Trigger '{}' 2>&1) || result=""
  if ! grep -Eq 'success=True' <<<"$result"; then
    echo "[错误] 视觉复位/搜索启动失败。"
    return 1
  fi
  echo "[启动] 已开始观测姿态复位；完成后自动进入紫色方块搜索。"
}

wait_for_zero_reset() {
  local deadline=$((SECONDS + 120)) result
  while (( SECONDS < deadline )); do
    result=$(timeout 3s ros2 topic echo --once \
      --qos-reliability reliable \
      --qos-durability transient_local \
      /brain_robot_visual_control/reason 2>&1) || result=""
    if grep -Fq 'RESET_COMPLETE' <<<"$result"; then
      echo "[复位] 六轴已到达软件零点。"
      return 0
    fi
    sleep 1
  done
  echo "[复位] 等待六轴零点反馈超时，未进入后续流程。"
  return 1
}

request_zero_reset() {
  local result
  result=$(timeout 8s ros2 service call /visual_search_controller/reset \
    std_srvs/srv/Trigger '{}' 2>&1) || result=""
  echo "$result"
  if ! grep -Eq 'success=True' <<<"$result"; then
    echo "[复位] 零点复位请求失败。"
    return 1
  fi
  wait_for_zero_reset
}

restart_visual_sequence() {
  echo "[重启] 停止当前视觉流程，先执行六轴零点复位..."
  timeout 5s ros2 service call /visual_search_controller/stop \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /servo_node/stop_servo \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  enable_physical_motion || return 1
  request_zero_reset || return 1
  start_visual_sequence
}

execute_grasp_sequence() {
  echo "[抓取] 请求抓取；当前状态必须为 GRASP_READY..."
  local result
  result=$(timeout 10s ros2 service call /grasp_lift_executor/execute \
    std_srvs/srv/Trigger '{}' 2>&1) || result=""
  echo "$result"
  if ! grep -Eq 'success=True' <<<"$result"; then
    echo "[抓取] 未启动：请确认视觉状态已经进入 GRASP_READY。"
    return 1
  fi
}

stop_motion_hold_enabled() {
  echo "[停止] 停止视觉和 Servo，保持 PiPER 使能与当前位置..."
  timeout 5s ros2 service call /visual_search_controller/stop \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /servo_node/stop_servo \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
}

reset_zero_sequence() {
  echo "[复位] 停止当前视觉运动，立即开始六轴零点复位..."
  timeout 5s ros2 service call /visual_search_controller/stop \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  timeout 5s ros2 service call /servo_node/stop_servo \
    std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true
  enable_physical_motion || return 1
  request_zero_reset
}

echo "=================================================="
echo " PiPER 实体紫色方块视觉抓取（一键保护模式）"
echo " 自动启动 CAN、PiPER 驱动、相机、MoveIt、Servo 和检测器"
echo "=================================================="
prepare_logs
bash "$SCRIPT_DIR/clean_disk_space.sh" >/dev/null 2>&1 || true
start_log_guard
echo "[1/7] 检查并初始化实体 CAN/机械臂..."
configure_can
if ! node_exists /piper_ctrl_single_node; then
  start_group piper_driver ros2 run piper piper_single_ctrl \
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
wait_for service /enable_srv

echo "[2/7] 检查并启动 RGB-D 相机..."
if ! node_exists /camera/camera; then
  start_group astra_camera ros2 launch astra_camera astra.launch.py
else
  echo "[复用] 已检测到 /camera/camera"
fi
wait_for topic /camera/color/image_raw
wait_for topic /camera/depth/image_raw

echo "[3/7] 启动紫色方块 RGB-D 检测..."
start_group cube_detector ros2 launch brain_robot_ball_pick cube_detector.launch.py
wait_for topic /brain_robot_vision/debug_image

echo "[4/7] 启动 MoveIt、Servo 和真机保护适配器..."
start_group visual_stack ros2 launch brain_robot_ball_pick cube_visual_search.launch.py
wait_for node /move_group
wait_for node /servo_node
wait_for node /piper_jog_adapter
wait_for node /grasp_lift_executor
wait_for service /piper_jog_adapter/arm
wait_for service /visual_search_controller/reset
wait_for service /grasp_lift_executor/execute

echo "[5/7] 检查完整 MoveIt 反馈..."
wait_for topic /piper_moveit_joint_states
echo "[6/7] 打开识别窗口..."
start_group debug_image_view ros2 run image_view image_view --ros-args -r image:=/brain_robot_vision/debug_image
echo "[7/7] 全部组件已就绪。"
echo
echo "操作：按 1 从六轴零点开始搜索/对齐；2=GRASP_READY 后抓取；0=仅零点复位；Ctrl+C=失能并退出。"
while true; do
  if [[ -t 0 ]] && read -r -s -n 1 -t 1 key; then
    case "$key" in
      1)
        restart_visual_sequence
        ;;
      2)
        execute_grasp_sequence
        ;;
      0)
        reset_zero_sequence
        ;;
    esac
  fi
done
