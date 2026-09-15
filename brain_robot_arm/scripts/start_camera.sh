#!/bin/bash
# 自动配置 Orbbec 相机 udev 规则并启动查看器

RULE_FILE=$(find /opt -name "99-obsensor-libusb.rules" 2>/dev/null | head -1)
if [ -z "$RULE_FILE" ]; then
    echo "错误：未找到 99-obsensor-libusb.rules 文件"
    exit 1
fi

echo "找到规则文件: $RULE_FILE"
sudo cp "$RULE_FILE" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "udev 规则已重新加载"

# 可选：等待 1 秒确保设备节点生成
sleep 1

# 启动 OrbbecViewer
OrbbecViewer
