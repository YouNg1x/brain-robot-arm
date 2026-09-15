from piper_sdk import C_PiperInterface_V2
import time

piper = C_PiperInterface_V2("can0")
piper.EnableArm(1)          # 1 = 使能
print("使能指令已发送")

# 可选：等待2秒，然后读取状态确认
time.sleep(2)
status = piper.GetArmStatus()
print(f"当前机械臂状态: {status}")

# 不发送任何运动指令，机械臂应保持当前位置但电机上电
