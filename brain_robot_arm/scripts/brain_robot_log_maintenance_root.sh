#!/usr/bin/env bash
# Root-only helper. It deliberately accepts no arguments so its sudoers entry
# cannot be repurposed to operate on arbitrary files.
set -Eeuo pipefail

if (( $# != 0 )); then
  echo "This maintenance helper accepts no arguments." >&2
  exit 2
fi

ROTATED_LOG_LIMIT_MB=200
JOURNAL_LIMIT_MB=200

find /var/log -maxdepth 1 -type f \
  \( -name 'syslog*' -o -name 'kern.log*' \) \
  -size "+${ROTATED_LOG_LIMIT_MB}M" -exec truncate -s 0 {} \;
journalctl --vacuum-size="${JOURNAL_LIMIT_MB}M"
