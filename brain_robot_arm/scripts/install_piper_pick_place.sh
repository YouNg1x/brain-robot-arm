#!/usr/bin/env bash
# 将共享文件中的取物演示复制到 Ubuntu、编译，并更新一键仿真启动脚本。

set -Eeuo pipefail

SHARED_ROOT="${BRAIN_ROBOT_SHARED_ROOT:-/mnt/hgfs/ub/brain_robot_arm}"
PACKAGE_SOURCE="$SHARED_ROOT/piper_ws/src/brain_robot_pick_place"
START_SCRIPT_SOURCE="$SHARED_ROOT/scripts/start_piper_sim.sh"
DEMO_SCRIPT_SOURCE="$SHARED_ROOT/scripts/start_brain_robot_demo.sh"
MONITOR_SCRIPT_SOURCE="$SHARED_ROOT/scripts/monitor_brain_robot_demo.sh"
PIPER_WS="${PIPER_ROS_WS:-$HOME/piper_ros}"
APP_WS="${BRAIN_ROBOT_WS:-$HOME/piper_ws}"
PACKAGE_TARGET="$APP_WS/src/brain_robot_pick_place"

fail() {
    echo "[错误] $*" >&2
    exit 1
}

[[ -z "${CONDA_PREFIX:-}" ]] || fail "请先执行 conda deactivate，再运行安装脚本。"
[[ -f /opt/ros/humble/setup.bash ]] || fail "没有找到 ROS2 Humble。"
[[ -f "$PIPER_WS/install/setup.bash" ]] || fail "请先编译 ~/piper_ros。"
[[ -f "$PACKAGE_SOURCE/package.xml" ]] || fail "共享文件夹中没有找到 brain_robot_pick_place。"
[[ -f "$START_SCRIPT_SOURCE" ]] || fail "共享文件夹中没有找到 start_piper_sim.sh。"
[[ -f "$DEMO_SCRIPT_SOURCE" ]] || fail "共享文件夹中没有找到 start_brain_robot_demo.sh。"
[[ -f "$MONITOR_SCRIPT_SOURCE" ]] || fail "共享文件夹中没有找到 monitor_brain_robot_demo.sh。"

mkdir -p "$PACKAGE_TARGET"
cp -a "$PACKAGE_SOURCE/." "$PACKAGE_TARGET/"
# cp -a preserves source mtimes.  On VMware shared folders that can make
# colcon reuse an older object file after a source update, so mark the copied
# package as freshly synchronized before building it.
find "$PACKAGE_TARGET" -type f -exec touch {} +
chmod +x "$PACKAGE_TARGET/scripts/"*.py

set +u
source /opt/ros/humble/setup.bash
source "$PIPER_WS/install/setup.bash"
set -u

ros2 pkg executables moveit_servo | grep -F 'servo_node_main' >/dev/null ||
    fail "缺少 MoveIt Servo。请先执行：sudo apt install ros-humble-moveit-servo"

cd "$APP_WS"
echo "[1/2] 正在编译 brain_robot_pick_place..."
colcon build --symlink-install --packages-select brain_robot_pick_place

echo "[2/2] 正在更新一键仿真启动脚本..."
cp "$START_SCRIPT_SOURCE" "$HOME/start_piper_sim.sh"
chmod +x "$HOME/start_piper_sim.sh"
cp "$DEMO_SCRIPT_SOURCE" "$HOME/start_brain_robot_demo.sh"
chmod +x "$HOME/start_brain_robot_demo.sh"
cp "$MONITOR_SCRIPT_SOURCE" "$HOME/monitor_brain_robot_demo.sh"
chmod +x "$HOME/monitor_brain_robot_demo.sh"

echo
echo "[完成] 场景包已安装。下一步运行："
echo "  bash ~/start_brain_robot_demo.sh"
echo "启动后可在第二个终端运行：bash ~/monitor_brain_robot_demo.sh"
echo "全部窗口就绪后，在同一个终端按 s 开始；按 x 停止；按一次 Ctrl+C 全部关闭。"
