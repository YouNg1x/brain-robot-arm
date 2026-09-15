# PiPER 多功能控制台

这是基于现有 `init_arm.sh`、`move_arm.py`、`move_to_points.py` 和复位脚本整合的图形控制台。原文件未被覆盖。

## 启动

在 Ubuntu 中先初始化 CAN 与机械臂：

```bash
conda activate base
bash /mnt/hgfs/ub/brain_robot_arm/scripts/init_arm.sh
```

再启动控制台：

```bash
python3 /mnt/hgfs/ub/brain_robot_arm/control_station/piper_control_station.py
```

如果提示缺少 Tkinter：

```bash
sudo apt install python3-tk
```

## 推荐测试顺序

1. 点击“连接 CAN0”。
2. 确认机械臂工作范围安全，点击“使能”。
3. 先测试夹爪，再测试很小的关节角度变化。
4. 确认末端坐标安全后测试“安全参考位”。
5. 最后才执行 XYZ 巡航演示。
6. 测试结束点击“失能”。

## 安全说明

- GUI 中的软件限位不是机械限位，也不能替代物理急停。
- 失能可能导致机械臂因重力下坠。
- 不要同时运行其他向 CAN0 写入运动指令的程序。
- 预设点来自旧脚本，不保证适合机械臂当前安装位置。
