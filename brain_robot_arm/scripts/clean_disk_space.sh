#!/usr/bin/env bash
# Safe disk-space cleanup for the PiPER ROS2 VM.
# Does not remove source code, ROS build/install directories, or conda packages.
set -Eeuo pipefail

THRESHOLD_PERCENT="${DISK_CLEAN_THRESHOLD:-85}"

usage_percent() {
  df --output=pcent / | tail -n 1 | tr -dc '0-9'
}

before="$(usage_percent)"
echo "清理前根分区使用率：${before}%"

echo "[1/3] 清理 ROS 日志..."
rm -rf "${HOME}/.ros/log"/* 2>/dev/null || true

echo "[2/3] 压缩 systemd 日志到 200 MB..."
sudo journalctl --vacuum-size=200M

echo "[3/3] 清理 apt 缓存..."
sudo apt clean

after="$(usage_percent)"
echo "清理后根分区使用率：${after}%"

if (( after >= THRESHOLD_PERCENT )); then
  echo "警告：根分区仍使用 ${after}%，未自动删除 ROS build/install/log 或 Conda 文件。"
  echo "如需进一步处理，请先检查：du -xhd1 /var /home 2>/dev/null | sort -h"
else
  echo "清理完成，当前低于 ${THRESHOLD_PERCENT}% 阈值。"
fi
