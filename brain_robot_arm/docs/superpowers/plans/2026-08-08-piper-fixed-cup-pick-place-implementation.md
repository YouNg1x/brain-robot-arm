# PiPER 固定杯子抓取演示实施计划

## 目标与边界

在现有 PiPER Gazebo + MoveIt2 仿真基础上，增加护理桌、标准圆柱杯和安全交付区，实现可分段验证的固定坐标抓取。所有新增代码放在独立工作区 `~/piper_ws`，不修改官方 `~/piper_ros`。第一版只允许仿真运行，不连接 CAN，不向真实机械臂发送指令。

Windows 共享文件中的唯一源代码目录为：

```text
C:\ub\brain_robot_arm\piper_ws\src\brain_robot_pick_place
```

Ubuntu 使用时复制到：

```text
~/piper_ws/src/brain_robot_pick_place
```

## 阶段一：场景和杯子跟随基础设施

1. 建立 `brain_robot_pick_place` ROS2 包，添加配置、模型、启动文件、Python 场景节点和 C++ Gazebo 杯子插件。
2. `scene_manager_node.py` 等待 Gazebo 的 `/spawn_entity` 服务，删除默认地面及旧的同名模型，按 YAML 参数生成降低后的地面、护理桌和杯子。机械臂保持 `(0,0,0)`，桌面上表面作为 `z=0`。
3. 同一个节点调用 MoveIt2 的 `/apply_planning_scene`，把桌面和杯子以碰撞物体形式加入 RViz。Gazebo 与 MoveIt2 共用同一份尺寸、位置参数，避免“看见的位置”和“规划的位置”不一致。
4. 杯子 SDF 加载 `libcup_pose_plugin.so`。插件只接收杯子目标位姿并在 Gazebo 更新线程中应用位姿，不依赖官方启动文件没有加载的 `/set_entity_state` 服务。
5. `cup_follow_node.py` 暂时只提供跟随开关和位姿发布能力；它使用 `world -> gripper_base` 的 TF 计算杯子跟随位置，为第二阶段夹取后同步移动做准备。

阶段一验收：Gazebo 中出现桌子和杯子；RViz 中 MotionPlanning 场景出现相同桌面和杯子；运行重置服务后两边恢复初始位置；机械臂仍可在 RViz 手动 Plan and Execute。

## 阶段二：自动抓取状态机

1. 增加 C++ `pick_place_node`，分别建立 `arm` 和 `gripper` 两个 MoveGroupInterface。
2. 启动时检查 `simulation_only=true`、控制器 action、`move_group` 和场景节点；任何一项缺失都拒绝运动。
3. 按 `HOME -> OPEN -> PREGRASP -> APPROACH -> CLOSE -> ATTACH -> LIFT -> TRANSFER -> LOWER -> RELEASE -> RETREAT -> DONE` 执行。
4. 大范围运动使用普通 MoveIt2 规划，接近、抬升和下降使用笛卡尔路径；笛卡尔完成比例低于 0.95 时停止。
5. 夹紧后，在 MoveIt2 中把杯子附着到 `gripper_base`，同时开启 Gazebo 杯子跟随。释放时关闭跟随并把杯子重新加入交付区的规划场景。
6. 提供 `stop_after` 参数，可停在 `pregrasp`、`approach`、`grasp`、`lift`、`transfer`、`place` 或 `done`，逐段实测。

阶段二验收：每一段可独立执行并停住；Gazebo 和 RViz 中杯子始终一致；规划失败时不会继续下一步；抓取后的失败保持夹爪闭合和杯子附着。

## 阶段三：项目接口预留

固定杯子抓取通过后，再把固定杯子位姿替换为 Gemini2 输出的目标位姿。SSVEP 用来选择场景中的目标，EOG 用来确认或取消，二者只发出高层任务命令，不直接持续控制每个关节。这样既能严格展示“SSVEP + EOG”，又能让 MoveIt2 继续负责避障、轨迹规划和执行安全。

## 验证顺序

每次只验证一层：先编译包，再显示 Gazebo 场景，再显示 RViz 碰撞场景，再测试杯子跟随，最后才运行自动抓取。Windows 侧只能完成文件、XML、YAML 和 Python 静态检查；ROS2 编译、Gazebo 插件加载、MoveIt2 规划和真实运动必须在 Ubuntu 中分别验证，不能把“代码存在”当作“实机已成功”。
