#!/bin/bash
# 机械臂一键初始化脚本（含 conda base 激活）
# 使用：bash ~/init_arm.sh

set -e

echo "=========================================="
echo "   Piper 机械臂初始化脚本"
echo "=========================================="

# 权限处理
if [ "$EUID" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# 1. 加载驱动并激活 can0
echo "[1/5] 重新加载 gs_usb 驱动..."
$SUDO ip link set can0 down 2>/dev/null || true
$SUDO modprobe -r gs_usb 2>/dev/null || true
$SUDO modprobe gs_usb
echo "驱动加载完成。"

echo "[2/5] 配置 can0 波特率 1Mbps 并启动..."
$SUDO ip link set can0 type can bitrate 1000000
$SUDO ip link set can0 up
echo "can0 已启动。"

echo "[3/5] 检查 can0 状态..."
if ip a show can0 | grep -q "state UP"; then
    echo "✓ can0 状态正常 (UP)"
else
    echo "✗ can0 状态异常，请检查 USB 连接"
    exit 1
fi

# 2. 激活 conda base 环境（关键）
echo "[4/5] 激活 conda base 环境..."
# 方式：初始化 conda 以便使用 conda activate
eval "$(conda shell.bash hook)"
conda activate base
# 可选：打印当前环境名称以确认
echo "✓ 当前 conda 环境: $(conda info --envs | grep '*' | awk '{print $1}')"

# 3. 双重使能机械臂
echo "[5/5] 使能机械臂（双重使能）..."
cd ~/piper_sdk/piper_sdk/demo/V2
python3 piper_ctrl_enable.py
sleep 0.5
python3 piper_ctrl_enable.py
echo "✓ 使能完成"

echo "=========================================="
echo "  初始化完成！可以开始控制机械臂。"
echo "=========================================="
