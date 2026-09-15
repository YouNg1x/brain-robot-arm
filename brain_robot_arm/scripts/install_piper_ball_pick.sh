#!/usr/bin/env bash
# Sync the red-ball task into Ubuntu and build the two affected ROS2 packages.
set -Eeuo pipefail

SHARED_ROOT="${BRAIN_ROBOT_SHARED_ROOT:-/mnt/hgfs/ub/brain_robot_arm}"
APP_WS="${BRAIN_ROBOT_WS:-$HOME/piper_ws}"
PIPER_WS="${PIPER_ROS_WS:-$HOME/piper_ros}"
BASE_SOURCE="$SHARED_ROOT/piper_ws/src/brain_robot_pick_place"
BALL_SOURCE="$SHARED_ROOT/piper_ws/src/brain_robot_ball_pick"
START_SOURCE="$SHARED_ROOT/scripts/start_brain_robot_ball_demo.sh"

fail() { echo "[错误] $*" >&2; exit 1; }
[[ -z "${CONDA_PREFIX:-}" ]] || fail "请先执行 conda deactivate。"
[[ -f /opt/ros/humble/setup.bash ]] || fail "未找到 ROS2 Humble。"
[[ -f "$PIPER_WS/install/setup.bash" ]] || fail "请先编译 ~/piper_ros。"
[[ -f "$BASE_SOURCE/package.xml" && -f "$BALL_SOURCE/package.xml" ]] || fail "共享文件夹缺少红球工程源文件。"
[[ -f "$START_SOURCE" ]] || fail "共享文件夹缺少红球一键启动脚本。"

mkdir -p "$APP_WS/src/brain_robot_pick_place" "$APP_WS/src/brain_robot_ball_pick"
cp -a "$BASE_SOURCE/." "$APP_WS/src/brain_robot_pick_place/"
cp -a "$BALL_SOURCE/." "$APP_WS/src/brain_robot_ball_pick/"
# VMware shared folders preserve mtimes; force colcon to see the synchronized sources.
find "$APP_WS/src/brain_robot_pick_place" "$APP_WS/src/brain_robot_ball_pick" -type f -exec touch {} +
chmod +x "$APP_WS/src/brain_robot_ball_pick/scripts/"*.py

set +u
source /opt/ros/humble/setup.bash
source "$PIPER_WS/install/setup.bash"
set -u
ros2 pkg executables moveit_servo | grep -F 'servo_node_main' >/dev/null ||
  fail "缺少 MoveIt Servo；先执行 sudo apt install ros-humble-moveit-servo"

cd "$APP_WS"
echo "[1/2] 编译通用视觉/抓取组件和红球工程..."
colcon build --symlink-install --packages-select brain_robot_pick_place brain_robot_ball_pick
echo "[2/2] 更新红球一键启动脚本..."
cp "$START_SOURCE" "$HOME/start_brain_robot_ball_demo.sh"
chmod +x "$HOME/start_brain_robot_ball_demo.sh"
echo "[完成] 运行：bash ~/start_brain_robot_ball_demo.sh；窗口齐全后按 s 开始。"
