#!/usr/bin/env python3
import time
from piper_sdk import C_PiperInterface_V2

# 步长（毫米）
STEP_X = 100
STEP_Y = 40      # Y方向步长（会配合X移动）
STEP_Z = 100

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
while not piper.EnablePiper():
    time.sleep(0.01)
piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)

def goto_reference():
    print("复位到参考点 (57,0,215,0,85,0)")
    piper.EndPoseCtrl(57000, 0, 215000, 0, 85000, 0)
    time.sleep(3)

goto_reference()

print("\n=== 末端位姿大幅度运动测试 ===")
print("命令: a/X+  d/X-  w/Y-  s/Y+  e/Z+  c/Z-  r/复位  q/退出")
print("注意: Y方向运动会同时调整X以确保可达性。")

while True:
    pose = piper.GetArmEndPoseMsgs()
    ep = pose.end_pose
    cur_x, cur_y, cur_z = ep.X_axis, ep.Y_axis, ep.Z_axis
    cur_rx, cur_ry, cur_rz = ep.RX_axis, ep.RY_axis, ep.RZ_axis
    print(f"\n当前位姿: X={cur_x/1000:.1f}  Y={cur_y/1000:.1f}  Z={cur_z/1000:.1f}  RY={cur_ry/1000:.1f}")
    
    cmd = input("输入方向: ").strip().lower()
    if cmd == 'q':
        break
    elif cmd == 'r':
        goto_reference()
        continue
    elif cmd == 'a':
        new_x, new_y, new_z = cur_x + STEP_X*1000, cur_y, cur_z
    elif cmd == 'd':
        new_x, new_y, new_z = cur_x - STEP_X*1000, cur_y, cur_z
    elif cmd == 'w':   # 负向Y移动（同时X辅助移动）
        new_x = cur_x - 30*1000
        new_y = cur_y - STEP_Y*1000
        new_z = cur_z
    elif cmd == 's':   # 正向Y移动（同时X辅助移动）
        new_x = cur_x + 30*1000
        new_y = cur_y + STEP_Y*1000
        new_z = cur_z
    elif cmd == 'e':
        new_x, new_y, new_z = cur_x, cur_y, cur_z + STEP_Z*1000
    elif cmd == 'c':
        new_x, new_y, new_z = cur_x, cur_y, cur_z - STEP_Z*1000
    else:
        print("无效命令")
        continue
    
    print(f"移动至: X={new_x/1000:.1f}, Y={new_y/1000:.1f}, Z={new_z/1000:.1f}")
    piper.EndPoseCtrl(new_x, new_y, new_z, cur_rx, cur_ry, cur_rz)
    time.sleep(3)
    
    pose = piper.GetArmEndPoseMsgs()
    ep = pose.end_pose
    print(f"实际到达: X={ep.X_axis/1000:.1f}, Y={ep.Y_axis/1000:.1f}, Z={ep.Z_axis/1000:.1f}")

print("演示结束，停止运动")
piper.MotionCtrl_2(0x00, 0x00, 0, 0x00)
