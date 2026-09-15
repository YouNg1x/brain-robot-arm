from piper_sdk import C_PiperInterface_V2
import time

piper = C_PiperInterface_V2("can0")
piper.EnableArm(1)
print("使能指令已发送，等待2秒...")
time.sleep(2)

# 查看使能状态
status = piper.GetArmStatus()
print(f"当前状态: {status}")

# 设置关节运动模式（参数：控制模式=0x01关节空间，运动模式=0x01位置，速度=30）
piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)

# 角度转换因子（弧度 -> 内部单位）
factor = 57295.7795
target_rad = [0, -0.3, 0, 0, 0, 0]
joint_0 = round(target_rad[0] * factor)
joint_1 = round(target_rad[1] * factor)
joint_2 = round(target_rad[2] * factor)
joint_3 = round(target_rad[3] * factor)
joint_4 = round(target_rad[4] * factor)
joint_5 = round(target_rad[5] * factor)

# 发送关节角度
piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
print("关节运动指令已发送")

# 夹爪闭合（参数：位置0~1000，速度1000，模式0x01，0）
piper.GripperCtrl(1000, 1000, 0x01, 0)
print("夹爪闭合指令已发送")

# 不需要 StopArm，运动完成后会自动停止
