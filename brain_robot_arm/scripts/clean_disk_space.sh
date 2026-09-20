#!/usr/bin/env bash
# Safe disk-space cleanup for the PiPER ROS2 VM.
# Does not remove source code, ROS build/install directories, or conda packages.
set -Eeuo pipefail

THRESHOLD_PERCENT="${DISK_CLEAN_THRESHOLD:-85}"
ROTATED_LOG_LIMIT_MB="${DISK_ROTATED_LOG_LIMIT_MB:-200}"
APP_LOG_DIR="${BRAIN_ROBOT_LOG_DIR:-$HOME/brain_robot_logs}"

usage_percent() {
  df --output=pcent / | tail -n 1 | tr -dc '0-9'
}

before="$(usage_percent)"
echo "清理前根分区使用率：${before}%"

echo "[1/4] 清理 ROS 日志..."
rm -rf "${HOME}/.ros/log"/* 2>/dev/null || true
find "$APP_LOG_DIR" -type f -name '*.log' -mtime +3 -delete 2>/dev/null || true
find "$APP_LOG_DIR" -type f -name '*.log' -size +100M -exec truncate -s 0 {} \; 2>/dev/null || true

echo "[2/4] 清理超大的轮转系统日志..."
# syslog and kern.log are ordinary rsyslog files, not systemd journals. They
# can grow to several GB, and journal vacuuming does not touch them. Truncate
# matching active/rotated files above the configured limit; keep the files and
# permissions so rsyslog/logrotate can continue normally.
sudo find /var/log -maxdepth 1 -type f \
  \( -name 'syslog*' -o -name 'kern.log*' \) \
  -size "+${ROTATED_LOG_LIMIT_MB}M" -exec sh -c '
    for log_file do
      printf "清空轮转日志：%s (%s)\n" "$log_file" "$(du -h "$log_file" | cut -f1)"
      truncate -s 0 "$log_file"
    done
  ' sh {} +

echo "[3/4] 压缩 systemd 日志到 200 MB..."
sudo journalctl --vacuum-size=200M

echo "[4/4] 清理 apt 缓存..."
sudo apt clean

after="$(usage_percent)"
echo "清理后根分区使用率：${after}%"

if (( after >= THRESHOLD_PERCENT )); then
  echo "警告：根分区仍使用 ${after}%，未自动删除 ROS build/install/log 或 Conda 文件。"
  echo "如需进一步处理，请先检查：du -xhd1 /var /home 2>/dev/null | sort -h"
else
  echo "清理完成，当前低于 ${THRESHOLD_PERCENT}% 阈值。"
fi
