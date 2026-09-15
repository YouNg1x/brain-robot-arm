#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动顺序运动（关节速度50，中等速度）：
1. 夹爪完全张开
2. 关节1 → 90°
3. 关节3 → -55°
4. 关节5 → 69.427°
5. 关节2 → 96.703°（到位后等待1秒）
6. 夹爪较慢速度闭合至20%
7. 原路返回零点，保持夹爪闭合20%
"""

import time
from piper_sdk import C_PiperInterface_V2

def deg2rad(deg):
    return deg * 3.1415926 / 180.0

def rad2internal(rad):
    return int(round(rad * 57295.7795))

MOVE_SPEED = 50          # 关节运动速度（1~100，50为中等速度）
GRIPPER_SPEED_SLOW = 200 # 夹爪较慢速度

def move_joints(piper, angles_deg, wait_sec):
    angles_rad = [deg2rad(a) for a in angles_deg]
    internals = [rad2internal(r) for r in angles_rad]
    piper.JointCtrl(*internals)
    time.sleep(wait_sec)

def main():
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    print("连接成功，正在使能...")
    for _ in range(3):
        piper.EnableArm(1)
        time.sleep(0.5)
    piper.MotionCtrl_2(0x01, 0x01, MOVE_SPEED, 0x00)
    print(f"使能完成，关节运动速度等级: {MOVE_SPEED}")

    print("\n=== 开始自动顺序运动 ===")

    # 1. 夹爪张开
    print("Step 1: 夹爪张开")
    piper.GripperCtrl(50000, 1000, 0x03, 0)
    time.sleep(2)

    # 2. 关节1 90°
    print("Step 2: 关节1 → 90°")
    move_joints(piper, [90, 0, 0, 0, 0, 0], wait_sec=3)   # 速度50，3秒足够

    # 3. 关节3 -55°
    print("Step 3: 关节3 → -55°")
    move_joints(piper, [90, 0, -55, 0, 0, 0], wait_sec=2)

    # 4. 关节5 69.427°
    print("Step 4: 关节5 → 69.427°")
    move_joints(piper, [90, 0, -55, 0, 69.427, 0], wait_sec=2)

    # 5. 关节2 96.703°（到位后额外等待1秒）
    print("Step 5: 关节2 → 96.703°")
    move_joints(piper, [90, 96.703, -55, 0, 69.427, 0], wait_sec=2)
    print("  等待关节2到位（1秒）...")
    time.sleep(1)

    # 6. 夹爪闭合至20%
    print("Step 6: 夹爪缓慢闭合至20%")
    piper.GripperCtrl(40000, GRIPPER_SPEED_SLOW, 0x03, 0)
    time.sleep(3)

    # 7. 原路返回零点（逆序），保持夹爪闭合
    print("Step 7: 原路返回零点（逆序）")
    print("  关节2回零...")
    move_joints(piper, [90, 0, -55, 0, 69.427, 0], wait_sec=2)
    print("  关节5回零...")
    move_joints(piper, [90, 0, -55, 0, 0, 0], wait_sec=2)
    print("  关节3回零...")
    move_joints(piper, [90, 0, 0, 0, 0, 0], wait_sec=2)
    print("  关节1回零...")
    move_joints(piper, [0, 0, 0, 0, 0, 0], wait_sec=3)

    print("\n所有步骤完成，机械臂已回到零点，夹爪保持闭合20%。")

if __name__ == "__main__":
    main()
