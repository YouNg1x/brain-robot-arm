# PiPER 实体紫色方块视觉任务状态

最后更新：2026-09-19

本文是项目的实机进展与交接记录。它与设计文档不同：设计文档描述方案；本文只记录已经由源码、构建结果或实际 ROS 运行日志支持的事实，并把推断和待验证事项分开。

## 当前目标与安全边界

当前目标是用实体 PiPER、Astra/Orbbec RGB-D 相机和 ROS 2：识别紫色正方体（`PURPLE_CUBE`），低速视觉搜索并把目标调整到画面中心，随后稳定进入 `GRASP_READY`。

当前阶段**不执行**自动接近、闭合夹爪或抬升。真实抓取动作只有在完成 TCP 标定、相机外参、碰撞范围、速度限制和空抓测试后，才可单独设计和启用。

本项目已经从早期的红杯、仿真红球实验演进而来；它们不能再作为当前紫色方块检测参数或真机抓取策略的依据。

## 已完成的重要节点

### 1. 仿真阶段

- 已完成 PiPER Gazebo、MoveIt2、RViz2、腕部相机和视觉检测的仿真环境搭建。
- 曾完成红色小球的仿真视觉搜索、居中和抓取流程。该流程验证了仿真中的感知、MoveIt 和抓取状态机组合，但不等同于真实机械臂轨迹已标定。
- 仿真抓取过程中曾验证视觉状态机包含搜索、目标重获、居中、整平/预抓取和抓取执行等阶段。

### 2. 真实硬件基础链路

- PiPER USB-CAN 适配器使用 `gs_usb` 驱动和 `can0`，工作波特率为 1 Mbps。
- 在 CAN 接口和实体连接正常时，`/joint_states` 已实测可稳定发布约 170--200 Hz。
- Astra/Orbbec ROS 2 驱动包已构建并能发布：
  - `/camera/color/image_raw`
  - `/camera/depth/image_raw`
  - `/camera/color/camera_info`
- OrbbecViewer 只能用于单独检查相机；它会占用设备，不能与 ROS 2 Astra 驱动同时使用。

### 3. 紫色方块感知与真机控制适配

- 已建立 `brain_robot_ball_pick` 中的紫色方块检测场景与配置。
- 实机日志曾出现 `PURPLE_CUBE target DETECTED`，证明紫色方块检测在至少部分运行中成功。
- 视觉控制器曾发布 `DIRECT_VISUAL_ALIGN_ACQUIRED`，说明检测目标已被视觉对准逻辑接收并满足过居中条件。
- 已实现 `piper_jog_adapter.py`：它订阅 MoveIt Servo 的 `/servo_node/delta_joint_cmds`，读取实体 `/joint_states`，生成受限速保护的 `/joint_commands`，由 `/piper_ctrl_single_node` 订阅。
- 已增加 `/piper_moveit_joint_states`，用于向 MoveIt 提供滤除实体驱动虚拟 `gripper` 字段、并补齐模型被动关节的关节状态。
- 手动启动适配器、调用 `enable_motion` 与 `arm` 后，已实测：
  - `/servo_node/delta_joint_cmds` 约 50 Hz；
  - `/joint_commands` 约 40--50 Hz；
  - `/joint_commands` 发布者为 `/piper_jog_adapter`，订阅者为 `/piper_ctrl_single_node`。

### 4. 真机 TF 诊断进展

- 已实测 `/piper_moveit_joint_states` 约 200 Hz，说明过滤后的关节反馈链正常。
- 已实测 `base_link -> gripper_base` 可以查询到，说明 `robot_state_publisher` 已能建立机器人本体链。
- 已确认此前的相机挂载错误：相机驱动已经发布 `camera_link -> camera_color_frame`，而项目又发布了 `gripper_base -> camera_color_frame`，导致 `camera_color_frame` 出现两个父节点并形成两棵 TF 树。
- 已修复启动文件，将固定外参改为 `gripper_base -> camera_link`。该修复已提交为 `48b1eb8`，需要在虚拟机重新编译并重启流程后验收。
- 重新编译并启动后，已实测 `gripper_base -> camera_color_optical_frame` 和 `base_link -> camera_color_optical_frame` 均能持续输出变换；TF 不再报告两棵不相连的树。开头偶发的 `Invalid frame ID` 出现在缓存刚启动阶段，随后查询成功。
- 已确认实体 PiPER ROS 驱动的正式使能服务名称是根命名空间 `/enable_srv`，不是 `/piper_ctrl_single_node/enable_srv`。调用 `piper_msgs/srv/Enable` 且 `enable_request: true` 后，实体机械臂开始执行此前由视觉适配器排队的运动命令。
- 实机首次真正运动时确认，驱动使能后会立即执行适配器当前已经积累的视觉目标；原来的 `0.035 rad/s` 对当前相机视野过快，可能使方块迅速离开画面。因此视觉搜索/居中速度和适配器限速已统一改为原来的五分之一：适配器 `0.007 rad/s`，搜索速度 `0.005/0.004 rad/s`，居中上限 `0.005 rad/s`。
- 已按仿真红球流程补强真机搜索策略：目标丢失时仍先进入 `LOCAL_SEARCH`，局部搜索失败后再进入完整搜索；完整搜索不再默认扫完整个关节范围，而是以本次启动时的人工观测姿态为中心，在 J1 ±0.12 rad、J5 ±0.10 rad 内低速扫描，并受绝对安全边界限制。
- `piper_jog_adapter` 现在在每次 `arm` 时记录视觉任务基线姿态；视觉 `JointJog` 只能使六轴相对该基线移动 `[0.12, 0.02, 0.02, 0.05, 0.10, 0.05]` rad。抓取轨迹仍走独立的轨迹通道，不受这条视觉搜索窗口限制，但仍受原有绝对关节范围和每周期限速保护。
- 根据用户确认，真机紫色方块任务已恢复仿真红球验证过的固定观测姿态和搜索边界：`[0.0, 0.98, -0.75, 0.0, 0.30, 0.0]`、J1 `[-0.98, 0.98]`、J5 `[-0.60, 0.95]`、局部 J5 搜索范围 `0.12`。真机搜索速度仍保持此前要求的五分之一。
- 真机 `PREPARE` 不再依赖不存在的 MoveIt 实体控制器：规划完成后通过 `/brain_robot_grasp/arm_trajectory` 交给 `piper_jog_adapter`，适配器按实体限速执行到仿真确认的观测姿态，反馈到位后才进入 `SEARCH`。

## 当前代码架构

关键启动链路如下：

```text
start_brain_robot_cube_real.sh
  -> PiPER 驱动（必要时）
  -> astra_camera（必要时）
  -> cube_detector.launch.py
  -> cube_visual_search.launch.py
       -> move_group
       -> servo_node
       -> piper_jog_adapter.py
       -> visual_search_controller
       -> grasp_lift_executor
  -> image_view（当前为 /brain_robot_vision/debug_image）
```

关键文件：

- `scripts/start_brain_robot_cube_real.sh`：实体一键启动、PID 组管理、按键交互和关闭流程。
- `piper_ws/src/brain_robot_ball_pick/launch/cube_visual_search.launch.py`：启动 MoveIt、Servo、适配器、视觉控制器和执行器。
- `piper_ws/src/brain_robot_ball_pick/config/cube_task_real.yaml`：真机紫色方块任务参数与保护模式。
- `piper_ws/src/brain_robot_pick_place/scripts/piper_jog_adapter.py`：Servo `JointJog` 到实体关节位置命令的受限转换。
- `piper_ws/src/brain_robot_pick_place/src/visual_search_controller.cpp`：视觉搜索、居中和状态机。
- `piper_ws/src/brain_robot_pick_place/config/piper_servo_real.yaml`：MoveIt Servo 真机配置。

## 当前控制行为

真实紫色方块配置处于保护模式。视觉控制开始后，目标偏离中心时应产生低速 Servo 修正；目标满足居中门槛后，控制器会进入 `DIRECT_VISUAL_ALIGN_ACQUIRED` 或 `GRASP_READY` 并停止持续修正。

实体驱动还有一层独立于视觉运动门的 ROS 内部使能状态：`/piper_jog_adapter` 的 `enable_motion`/`arm` 只控制保护适配器，不能替代 `/enable_srv` 对 `piper_ctrl_single_node` 的使能。未调用 `/enable_srv` 时，`/joint_commands` 仍可有数据，但驱动回调不会调用 `JointCtrl()`，实体机械臂不会运动。

因此，目标已经居中时没有明显关节运动、或 `/joint_commands` 在控制器停止输出后不再持续发布，可能是当前设计行为，而不自动表示故障。

## 已识别的故障与处理历史

### 启动和进程生命周期不稳定

- 有些一键启动后的运行中，`/piper_jog_adapter` 不存在；此时即使 Servo 有输出，也不会有节点把它转为 `/joint_commands`。
- 直接运行 `ros2 run brain_robot_pick_place piper_jog_adapter.py` 能在部分测试中启动适配器，表明需要比较该默认启动与 `cube_visual_search.launch.py` 中参数化启动的差异。
- `start_brain_robot_cube_real.sh` 已使用 `setsid` 和 PID 组记录本脚本启动的顶层进程；不能把“未保存后台 PID”当作未经验证的根因。

### 重复节点与 ROS 图残留

- 历史运行中曾出现同名的 `cube_detector`、`servo_node`、`visual_search_controller` 或 `move_group`。
- `ros2 node list` 的重复条目不总是代表真实双进程：曾出现进程已结束但 ROS daemon/图仍显示旧节点的情况。
- 单实例判断必须同时检查 OS 进程、ROS 节点图和 ROS daemon 重启后的状态；仅凭 `ros2 node list` 不能直接下结论。

### 相机与 CAN 的外部条件

- OrbbecViewer 与 Astra ROS 驱动不能同时占用相机。
- USB 连接不稳定曾造成相机节点无法找到 UVC 彩色设备。
- CAN 适配器或机械臂连接松动曾造成 `can0 is loss`、无反馈或无 CAN 报文；这属于硬件/接口状态，不应误判为视觉或 Servo 算法故障。

### 早期 Servo 模型状态问题

- 实体驱动的 `JointState` 曾包含模型中不存在的 `gripper`，且 MoveIt 模型还需要被动关节状态；这曾引发 Servo 因关节模型不完整而退出。
- 适配器的过滤关节状态话题用于隔离这一差异。后续每次重构都必须保留此数据适配层。

## 当前验证标准

### 操作者可见界面

一键启动后只显示检测调试图：`/brain_robot_vision/debug_image`，用于确认检测框、目标中心和识别状态。原始彩色图窗口已按操作者要求移除；不能只通过 `ros2 topic hz` 判断视觉功能已经可用。

### 控制数据链路

在目标偏离画面中心、运动门开启、适配器已 arm 的情况下，按以下顺序验证：

```text
/joint_states
  -> /piper_moveit_joint_states
  -> /servo_node/delta_joint_cmds
  -> /joint_commands
  -> /joint_states 的实际角度变化
```

其中前四个话题存在或有频率并不单独证明实体机械臂运动；最后必须以反馈关节角度发生预期变化作为实机验证证据。

## 本轮已实现、待真机验收

本轮新增的搜索保护尚待真机验收。重新启动前必须先把机械臂从上次 `MOVEIT_SERVO_HARD_STOP_5` 停留姿态人工恢复到安全观测姿态；否则“以当前姿态为中心”的保护窗口会围绕错误姿态工作，不能替代机械复位。

1. 抓取执行器新增显式 `/grasp_lift_executor/execute` 服务；只有当前视觉状态为 `GRASP_READY` 且真机抓取授权参数开启时才启动。
2. 真机轨迹通过 `/brain_robot_grasp/arm_trajectory` 交给 `piper_jog_adapter`，不再调用被禁用的 MoveIt 实体 `execute()`。
3. 真机夹爪通过 `/brain_robot_grasp/gripper_command` 复用 `auto_sequence_speed50.py` 中的 `50000` 张开、`40000` 收紧值。
4. 一键脚本新增按键 `6` 触发抓取，只打开检测调试图窗口。

这些改动已完成源码级检查；真机抓取仍未完成最终验收。最近一轮实机 TF 输出已证明机器人反馈正常，但相机挂载 TF 在修复前仍断链；修复提交后必须重新编译、重启并再次查询完整链路。

## 当前未完成事项

1. 重新编译并在真机上验证五分之一低速配置，确认目标不会因修正过快而离开视野。
2. 验证 `50000/40000` 对当前实体夹爪的开合效果。
3. 验证紫色方块偏离时的真机低速视觉修正，以及居中后停止。
4. 完成以上步骤后，才进行一次显式 `6` 键抓取测试；不改为目标稳定后自动闭爪。

## 推荐安全操作顺序

1. 清空机械臂工作区并确认急停/手动断电方式可用。
2. 只启动一套实体流程。
3. 先观察两个图像窗口和实体反馈，再开启运动门。
4. 开启运动门、arm 适配器后，才启动视觉搜索。
5. 发生异常时先停止视觉控制，再 disarm/关闭运动门，最后结束相关进程。

### 实体驱动使能命令

启动流程后，在确认工作区安全时执行一次：

```bash
timeout 8s ros2 service call \
  /enable_srv \
  piper_msgs/srv/Enable \
  "{enable_request: true}"
```

必须返回 `enable_response: true`。这一步完成后，才按 `1`、`2`、`3` 进行保护门、适配器和视觉搜索操作。停止时先按 `4`、`5`，再按 `0`。

## 可直接执行的真机启动流程

以下命令在虚拟机终端执行。启动前确认相机 USB、CAN 适配器和机械臂接口已连接；OrbbecViewer 必须关闭，避免占用相机。

### 1. 编译并安装当前版本

```bash
conda deactivate
bash /mnt/hgfs/ub/brain_robot_arm/scripts/install_piper_cube_pick.sh
```

该脚本会把共享目录中的两个 ROS 包复制到 `~/piper_ws`，重新编译，并安装一键脚本到 `~/start_brain_robot_cube_real.sh`。本次 TF 修复必须重新执行这一步后才会进入已安装的 launch 文件。

### 2. 启动实体保护流程

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/piper_ws/install/setup.bash
bash ~/start_brain_robot_cube_real.sh
```

启动脚本会依次检查或启动 CAN/PiPER 驱动、Astra RGB-D 相机、紫色方块检测器、MoveIt、Servo、真机保护适配器、视觉控制器、抓取执行器，并只打开 `/brain_robot_vision/debug_image` 检测窗口。

### 3. 启动后的按键顺序

```text
1  开启运动门
2  使能并 arm 适配器
3  自动搜寻紫色方块并居中
6  仅当状态为 GRASP_READY 且现场安全时执行一次抓取
4  停止视觉运动
5  失能适配器
0  退出并自动关闭运动门
```

### 4. 按 6 前的最低验证

在按 `6` 前，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/piper_ws/install/setup.bash

timeout 8s ros2 run tf2_ros tf2_echo \
  gripper_base camera_color_optical_frame
```

必须持续输出变换，且不能出现 `two or more unconnected trees`。同时检测窗口应显示紫色方块，状态话题应为 `GRASP_READY`。如果 TF、反馈或现场安全任一项不满足，先按 `4`、`5`，不要按 `6`。
