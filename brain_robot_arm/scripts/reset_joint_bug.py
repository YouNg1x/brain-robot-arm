from piper_sdk import C_PiperInterface_V2
import time

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()

print("1. 切换到主模式...")
piper.MotionCtrl_1(0x01, 0, 0)
time.sleep(0.5)

print("2. 切换回从模式...")
piper.MotionCtrl_1(0x02, 0, 0)
time.sleep(0.5)

print("3. 执行复位...")
piper.MotionCtrl_1(0x00, 0, 0)
print("复位完成")
