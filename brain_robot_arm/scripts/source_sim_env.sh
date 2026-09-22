#!/usr/bin/env bash

BRAIN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export BRAIN_ROOT
export PIPER_ROS_WS="$BRAIN_ROOT/vendor/piper_ros"
export BRAIN_ROBOT_WS="$BRAIN_ROOT/piper_ws"
export PIPER_SIM_SCRIPT="$BRAIN_ROOT/scripts/start_piper_sim.sh"

# ROS 2 Humble
source /opt/ros/humble/setup.bash

# PiPER 底层工作空间
if [ -f "$PIPER_ROS_WS/install/setup.bash" ]; then
    source "$PIPER_ROS_WS/install/setup.bash"
else
    echo "[警告] piper_ros 尚未编译：$PIPER_ROS_WS"
fi

# 脑控机械臂工作空间
if [ -f "$BRAIN_ROBOT_WS/install/setup.bash" ]; then
    source "$BRAIN_ROBOT_WS/install/setup.bash"
else
    echo "[警告] piper_ws 尚未编译：$BRAIN_ROBOT_WS"
fi

# Gazebo / RViz 在 Ubuntu 图形环境下优先使用 X11/XWayland
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "=========================================="
echo " Brain Robot Arm 仿真环境已加载"
echo "=========================================="
echo "BRAIN_ROOT     = $BRAIN_ROOT"
echo "PIPER_ROS_WS   = $PIPER_ROS_WS"
echo "BRAIN_ROBOT_WS = $BRAIN_ROBOT_WS"
echo "ROS_DISTRO     = $ROS_DISTRO"
