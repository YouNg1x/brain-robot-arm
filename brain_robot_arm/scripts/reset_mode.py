from piper_sdk import C_PiperInterface_V2
import time

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
print("已连接")

piper.MotionCtrl_1(0x01, 0, 0)
print("切换到主模式")
time.sleep(1)

piper.MotionCtrl_1(0x02, 0, 0)
print("切换回从模式")
time.sleep(1)

piper.MotionCtrl_1(0x00, 0, 0)
print("执行重置")
time.sleep(1)

piper.EnableArm(1)
print("尝试使能")

status = piper.GetArmStatus()
print("当前状态:", status)
