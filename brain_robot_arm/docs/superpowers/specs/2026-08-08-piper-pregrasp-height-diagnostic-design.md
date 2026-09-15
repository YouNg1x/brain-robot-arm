# PiPER 预抓取高度单变量诊断设计

日期：2026-08-08  
适用范围：Gazebo + MoveIt2 仿真环境中的固定杯子预抓取规划  
当前阶段：仅规划（PLAN-ONLY），禁止执行轨迹

## 1. 问题与证据

当前场景、MoveIt2 服务及碰撞物体均已正常加载。预抓取节点能够读取杯子中心
`(0.400, -0.120, 0.050)`，并生成夹爪参考点目标
`(0.200, -0.120, 0.090)`，但 OMPL 在规划时间内无法为目标树采样到合法状态，最终报告
`Unable to sample any valid states for goal tree`。这说明故障发生在目标位姿的逆运动学或碰撞有效性阶段，
而不是节点未连接、场景未就绪或控制器未启动。

当前目标高度按以下关系得到：

```text
target_z = cup_z + gripper_vertical_offset
         = 0.05 + 0.04
         = 0.09 m
```

该目标明显低于机械臂零位时 `gripper_base` 的近似高度。保持当前夹爪姿态时，低位目标可能造成
夹爪、腕部或连杆与桌面碰撞，也可能使逆运动学无解。因此先单独提高目标高度，判断低位约束是否是
规划失败的主要原因。

## 2. 设计目标

本次只改变 `gripper_vertical_offset`，不改变杯子位置、水平接近距离、夹爪姿态、规划器、碰撞场景
和零位检查。默认将垂直偏移从 `0.04 m` 调整为 `0.12 m`，对应的新目标为：

```text
target = (0.200, -0.120, 0.170) m
```

节点仍以 `execute=false` 启动，只生成并显示规划轨迹，不向 Gazebo 控制器发送轨迹。这样可以在不产生
机械臂运动的情况下，单独验证目标高度对逆解和碰撞检查的影响。

## 3. 参数化方案

在 `pregrasp.launch.py` 中增加启动参数 `vertical_offset`，默认值为 `0.12`。启动文件把该值覆盖到
节点参数 `gripper_vertical_offset`，底层 C++ 的目标计算公式保持不变。保留 `scene.yaml` 中的参数作为
基础配置，启动参数用于诊断时快速覆盖。

默认测试命令预期为：

```bash
ros2 launch brain_robot_pick_place pregrasp.launch.py
```

需要单变量复测时可以显式指定高度，例如：

```bash
ros2 launch brain_robot_pick_place pregrasp.launch.py vertical_offset:=0.10
ros2 launch brain_robot_pick_place pregrasp.launch.py vertical_offset:=0.12
ros2 launch brain_robot_pick_place pregrasp.launch.py vertical_offset:=0.14
```

启动参数单位统一为米。首轮只测试默认的 `0.12 m`；只有默认值仍失败时，才按一次一个数值的方式继续
测试，避免同时改变多个变量后无法判断原因。

## 4. 不变项与安全边界

- `execute` 默认值保持 `false`，本轮不得使用 `execute:=true`。
- 杯子中心保持 `(0.400, -0.120, 0.050)`。
- 预抓取水平参数保持 `pregrasp_distance=0.10`、`gripper_grasp_offset=0.10`。
- 夹爪方向继续使用当前 `gripper_base` 姿态，不进行旋转。
- 零位检查保持启用，容差保持 `0.08 rad`。
- 桌子与杯子的 MoveIt2 碰撞模型保持不变。
- 规划时间、尝试次数、速度和加速度比例均保持不变。
- 本结果只代表 Gazebo/MoveIt2 仿真规划验证，不代表真实机械臂已经安全可执行。

## 5. 预期日志与验收标准

启动后应先看到目标变为：

```text
Cup center (0.400, -0.120, 0.050); gripper_base pregrasp target (0.200, -0.120, 0.170).
```

本轮通过条件为 MoveIt2 返回规划成功，并且 RViz2 中出现不穿过桌面和杯子的预览轨迹。Gazebo 中的
机械臂应保持不动，因为 `execute=false`。若仍出现 `Unable to sample any valid states for goal tree`，则
说明仅提高高度不足以恢复合法目标，下一轮应继续在 PLAN-ONLY 模式下诊断目标姿态或水平位置，不能
直接开启执行。

## 6. 实施范围

批准实施后仅修改：

```text
piper_ws/src/brain_robot_pick_place/launch/pregrasp.launch.py
```

并运行 Python 语法检查、项目现有静态测试以及 Ubuntu 侧重新构建。Windows 侧检查和 Ubuntu 编译成功
都不能代替 Gazebo 中的实际规划验证；最终仍需由用户在三个终端均正常运行的环境中执行一次默认
PLAN-ONLY 测试并提供日志。
