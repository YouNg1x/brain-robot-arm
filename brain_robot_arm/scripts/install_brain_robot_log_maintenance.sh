#!/usr/bin/env bash
# One-time installation of a deliberately narrow passwordless log-maintenance helper.
set -Eeuo pipefail

HELPER_SOURCE="${HOME}/brain_robot_log_maintenance_root.sh"
HELPER_TARGET="/usr/local/sbin/brain-robot-log-maintenance"
SUDOERS_TARGET="/etc/sudoers.d/brain-robot-log-maintenance"
ACCOUNT_NAME="${USER}"

[[ -f "$HELPER_SOURCE" ]] || {
  echo "[错误] 未找到 $HELPER_SOURCE；请先运行安装脚本更新文件。" >&2
  exit 1
}
[[ "$ACCOUNT_NAME" =~ ^[a-z_][a-z0-9_-]*$ ]] || {
  echo "[错误] 当前账户名不符合 sudoers 安全格式。" >&2
  exit 1
}

echo "[说明] 此操作需要输入一次当前账户密码。"
echo "[说明] 之后仅允许免密执行固定的系统日志维护助手；不能传入参数，也不能获得通用 sudo 权限。"
sudo install -o root -g root -m 0755 "$HELPER_SOURCE" "$HELPER_TARGET"
printf '%s ALL=(root) NOPASSWD: %s ""\n' "$ACCOUNT_NAME" "$HELPER_TARGET" |
  sudo tee "$SUDOERS_TARGET" >/dev/null
sudo chmod 0440 "$SUDOERS_TARGET"
sudo visudo -cf "$SUDOERS_TARGET"
sudo -n "$HELPER_TARGET"
echo "[完成] 已安装受限免密日志维护。以后可直接运行 ~/clean_disk_space.sh。"
