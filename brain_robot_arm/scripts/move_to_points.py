#!/usr/bin/env python3
from piper_sdk import C_PiperInterface_V2
import time

def main():
    # 初始化
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    print("使能机械臂...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("使能成功")

    # 夹爪张开（方便观察末端运动）
    piper.GripperCtrl(50000, 1000, 0x03, 0)
    time.sleep(1)

    # 设置运动模式为末端位姿控制
    # 参数: 0x01（关节空间运动）, 0x00（末端位姿模式）, 速度30, 0x00
    piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
    print("末端位姿控制模式已激活")

    # 定义目标点（单位：毫米，姿态：毫度）
    # 格式: [x, y, z, rx, ry, rz]
    # 官方示例中 ry=85° 对应 85000 毫度
    points = [
        [ 57,   0, 215, 0, 85000, 0],   # 起始参考点
        [157,   0, 215, 0, 85000, 0],   # X +100mm
        [ 57, 100, 215, 0, 85000, 0],   # Y +100mm（可能不可达，测试用）
        [ 57,   0, 315, 0, 85000, 0],   # Z +100mm
        [ 57,   0, 215, 0, 85000, 0],   # 回到起始点
    ]

    print("\n将依次运动到以下点位，按回车继续...")
    for i, p in enumerate(points):
        x, y, z, rx, ry, rz = p
        print(f"\n点 {i+1}: X={x}mm, Y={y}mm, Z={z}mm, 姿态固定({rx},{ry},{rz})")
        input("按回车开始运动...")
        piper.EndPoseCtrl(x, y, z, rx, ry, rz)
        time.sleep(3)   # 等待运动完成

    print("\n运动完成，停止机械臂")
    piper.MotionCtrl_2(0x00, 0x00, 0, 0x00)

if __name__ == "__main__":
    main()
