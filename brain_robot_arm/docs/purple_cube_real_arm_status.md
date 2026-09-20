# PiPER 实体紫色方块视觉任务状态

最后更新：2026-09-20

本文是项目的实机进展与交接记录。它与设计文档不同：设计文档描述方案；本文只记录已经由源码、构建结果或实际 ROS 运行日志支持的事实，并把推断和待验证事项分开。

## 当前目标与安全边界

当前目标是用实体 PiPER、Astra/Orbbec RGB-D 相机和 ROS 2：启动后自动复位，目标暂时不可见时自动搜索紫色正方体（`PURPLE_CUBE`），检测到后低速居中并自动执行抓取。

当前版本已按用户确认启用 `GRASP_READY` 后自动执行预抓取、接近、闭爪和抬升；`6` 只保留为停止后的手动重试入口。TCP、相机外参、碰撞范围和空抓结果仍需在实体上继续记录，作为后续调参依据。

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
- 实机首次真正运动时确认，驱动使能后会立即执行适配器当前已经积累的视觉目标；原来的 `0.035 rad/s` 对当前相机视野过快，曾先统一降到五分之一。2026-09-20 根据最新实测需求，当前配置再次整体提高为上一版的三倍：适配器 `0.189 rad/s`，搜索速度 `0.135/0.108 rad/s`，居中上限 `0.135 rad/s`，水平/垂直居中增益同步改为 `0.00675`，水平锁定上限改为 `0.081 rad/s`。
- 准备轨迹接收后，适配器现在把轨迹终点作为视觉基线；J1/J5 的视觉限幅因此围绕仿真观测姿态生效，不会再被启动前的旧姿态截断。J2/J3 只在 PREPARE 进入观测姿态，SEARCH/ALIGN 阶段不再被重新写入。
- 适配器现在在每次成功调用 `/piper_jog_adapter/arm` 时把实体夹爪目标设为 `0.05`（对应 SDK 的 `50000` 张开值），并在后续周期持续发布；这样启动时不会依赖可能丢失的一次性 ROS 夹爪消息，也不会以抓取阶段的闭合值启动。
- 已按仿真红球流程补强真机搜索策略：目标丢失时先进入 `LOCAL_SEARCH`，局部搜索失败后直接从当前姿态进入完整搜索；完整搜索一轮结束后继续下一轮，不再回到 `PREPARE` 复位。完整搜索不再默认扫完整个关节范围，而是按配置范围低速扫描并受绝对关节边界限制。
- 目标在 `ALIGN` 阶段丢失时，控制器会先进入 `LAST_PATH_REACQUIRE`：保存最近一次非零 J1/J5 视觉控制方向，沿该方向分别延伸 0.20/0.15 rad；期间若发现紫色候选则优先转向候选。候选仍必须通过原有正方体形状、尺寸和深度确认，确认失败后才进入局部搜索。
- `piper_jog_adapter` 现在在每次 `arm` 时记录视觉任务基线姿态；视觉 `JointJog` 只能使六轴相对该基线移动 `[0.12, 0.02, 0.02, 0.05, 0.10, 0.05]` rad。抓取轨迹仍走独立的轨迹通道，不受这条视觉搜索窗口限制，但仍受原有绝对关节范围和每周期限速保护。
- 根据用户确认，真机紫色方块任务使用固定观测姿态 `[0.0, 0.98, -0.75, 0.0, 0.30, 0.0]`；当前扩大后的 J1 搜索范围为 `[-1.40, 1.40]`，J5 为 `[-0.60, 0.95]`，局部 J5 搜索范围 `0.12`。
- 真机 `PREPARE` 不再依赖不存在的 MoveIt 实体控制器：规划完成后通过 `/brain_robot_grasp/arm_trajectory` 交给 `piper_jog_adapter`，适配器按实体限速执行到仿真确认的观测姿态，反馈到位后才进入 `SEARCH`。
- 实体模式的 `grasp_lift_executor` 不再创建不存在的 MoveIt `gripper` 规划组；实体夹爪只通过 `/brain_robot_grasp/gripper_command` 发送 `50000/40000`，仿真模式仍保留 MoveIt 夹爪组。这样消除了反复出现的 `Joint 'gripper' not found in model 'piper'` 错误。
- `grasp_lift_executor` 也必须像 `move_group` 一样订阅 `/piper_moveit_joint_states`；若直接订阅实体 `/joint_states`，驱动附带的 `gripper` 字段会反复被 MoveIt 拒绝，并污染抓取前状态。启动文件已补齐该重映射。
- 一键脚本现在把驱动、相机、检测器、视觉栈和调试窗口输出写入 `~/brain_robot_logs/`，不再把高频日志刷满启动终端；启动时会清理超过 3 天或单个超过 100 MB 的应用日志。`~/clean_disk_space.sh` 也会执行同样的应用日志清理。
- 2026-09-20 已在实体机完成关键链路验证：一键启动后按 `1、2、3`，控制器发布约 49 秒的复位轨迹，适配器成功接收并持续输出 `/joint_commands` 约 50 Hz，机械臂已实际进入自动搜寻模式。
- 同次验证发现并修复适配器在轨迹回调中使用 ROS 2 Python 日志位置参数导致的 `TypeError`；修复提交为 `4d3ce03`。此前该异常会使适配器退出，表现为有轨迹发布但没有 `/joint_commands`。
- 当前一键脚本只保留两个操作键：`1` 自动使能并启动“复位→搜寻→居中→抓取”，`2` 停止视觉与 Servo 当前运动但保持 PiPER 和适配器使能；`0` 退出并执行完整关闭流程。

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

- `scripts/start_brain_robot_cube_real.sh`：实体一键启动 CAN、PiPER 驱动、相机、检测器、MoveIt/Servo、适配器、PID 组管理、按键交互和关闭流程；不再依赖单独的 `init_arm.sh`。
- `piper_ws/src/brain_robot_ball_pick/launch/cube_visual_search.launch.py`：启动 MoveIt、Servo、适配器、视觉控制器和执行器。
- `piper_ws/src/brain_robot_ball_pick/config/cube_task_real.yaml`：真机紫色方块任务参数与保护模式。
- `piper_ws/src/brain_robot_pick_place/scripts/piper_jog_adapter.py`：Servo `JointJog` 到实体关节位置命令的受限转换。
- `piper_ws/src/brain_robot_pick_place/src/visual_search_controller.cpp`：视觉搜索、居中和状态机。
- `piper_ws/src/brain_robot_pick_place/config/piper_servo_real.yaml`：MoveIt Servo 真机配置。

## 当前控制行为

真实紫色方块配置处于保护模式。视觉控制开始后，目标偏离中心时应产生低速 Servo 修正；目标满足居中门槛后，控制器会进入 `DIRECT_VISUAL_ALIGN_ACQUIRED` 或 `GRASP_READY` 并停止持续修正。

### 实体任务的正确状态流程

每次启动实体流程后，操作者不需要先把方块放到画面中心。按键 `1` 会自动使能并执行仿真红球已经验证过的固定复位/观测姿态：

```text
PREPARE
  -> [0.0, 0.98, -0.75, 0.0, 0.30, 0.0]
SEARCH
  -> 画面暂时没有紫色方块时自动搜索
ALIGN
  -> 方块进入视野后自动居中
  GRASP_READY
  -> 自动执行一次抓取；按键 6 仅用于停止后手动重试
```

因此复位后看到天花板或暂时看不到方块并不意味着流程失败；只要已经进入 `SEARCH`，控制器就会按仿真红球的 J1/J5 搜索策略寻找目标。只有在搜索边界完成并发布 `FULL_SEARCH_COMPLETE_NO_TARGET` 后，才需要检查相机视野、目标摆放和复位姿态。

实体驱动还有一层独立于视觉运动门的 ROS 内部使能状态：`/piper_jog_adapter` 的 `enable_motion`/`arm` 只控制保护适配器。现在一键脚本的按键 `2` 会先调用 `/enable_srv` 使能 `piper_ctrl_single_node`，成功后才 arm 保护适配器；这样不会再出现“适配器已 arm、`/joint_commands` 有数据、但实体驱动未使能”的混淆状态。

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

### 本次 CAN/启动故障的处理方法

- `SendCanMessage(SEND_MESSAGE_FAILED (100017))` 和 `can0 is loss` 表示 CAN 发送层失败。先停止 ROS 驱动，确认 PiPER 电源、CANH/CANL、USB-CAN 和虚拟机 USB 接管，再重新初始化 `can0` 为 1 Mbps；不能靠重复调用 `/enable_srv` 修复。
- `can0` 重新启动后，必须先看到 `/joint_states` 稳定约 170--200 Hz，再启动视觉流程。只有 USB 识别到 CAN 适配器不代表机械臂总线已经连通。
- `/piper_jog_adapter` 消失时，即使 Servo 或视觉节点还在，实体也不会运动。启动脚本现在把适配器作为必需节点等待；手动诊断时必须确认节点和 `arm` 服务都存在。
- `START_AUTHORIZED` 只表示视觉启动请求已接受；`PREPARE` 表示正在复位；`SEARCH` 才表示开始自动寻找目标；`PREPARE_EXECUTION_FAILED` 表示准备姿态没有通过实体反馈到位。
- 本次曾因机械臂停在 `joint1≈1.498、joint5≈-1.199` 的硬停姿态而无法执行准备轨迹。正确处理是先停止视觉、失能 PiPER，再人工恢复到安全姿态，之后重新启动流程。

### 早期 Servo 模型状态问题

- 实体驱动的 `JointState` 曾包含模型中不存在的 `gripper`，且 MoveIt 模型还需要被动关节状态；这曾引发 Servo 因关节模型不完整而退出。
- 适配器的过滤关节状态话题用于隔离这一差异。后续每次重构都必须保留此数据适配层。

## 当前验证标准

### 操作者可见界面

一键启动后只显示检测调试图：`/brain_robot_vision/debug_image`，用于确认检测框、目标中心和识别状态。原始彩色图窗口已按操作者要求移除；不能只通过 `ros2 topic hz` 判断视觉功能已经可用。

### 真正的一键启动

当前脚本会在同一个终端内完成以下编排：重载 `gs_usb`、配置 `can0` 为 1 Mbps、启动 PiPER 驱动并等待 `/joint_states`，启动 Astra RGB-D 相机并等待图像话题，然后启动紫色方块检测、MoveIt、Servo、保护适配器、视觉控制器、抓取执行器和调试窗口。脚本退出时会停止本次启动的进程并关闭运动门。以后不需要分别打开驱动、相机和视觉三个终端；安装脚本会把它更新为 `~/start_brain_robot_cube_real.sh`。

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

本轮新增的最后轨迹重捕获和紫色候选重捕获尚待真机验收。重新启动前必须先把机械臂从上次 `MOVEIT_SERVO_HARD_STOP_5` 停留姿态人工恢复到安全观测姿态；否则“以当前姿态为中心”的保护窗口会围绕错误姿态工作，不能替代机械复位。

1. 抓取执行器新增显式 `/grasp_lift_executor/execute` 服务；只有当前视觉状态为 `GRASP_READY` 且真机抓取授权参数开启时才启动。
2. 真机轨迹通过 `/brain_robot_grasp/arm_trajectory` 交给 `piper_jog_adapter`，不再调用被禁用的 MoveIt 实体 `execute()`。
3. 真机夹爪通过 `/brain_robot_grasp/gripper_command` 复用 `auto_sequence_speed50.py` 中的 `50000` 张开、`40000` 收紧值。
4. 一键脚本现在只保留数字键 `1` 和 `2`：`1` 一键使能并启动完整自动抓取；`2` 停止当前视觉/Servo 运动但保持 PiPER 和适配器使能；`0` 退出并完整关闭。

这些改动已完成源码级检查；真机抓取仍未完成最终验收。最近一轮实机 TF 输出已证明机器人反馈正常，但相机挂载 TF 在修复前仍断链；修复提交后必须重新编译、重启并再次查询完整链路。

## 当前未完成事项

1. 重新编译并在真机上验证五分之一低速配置，确认目标不会因修正过快而离开视野。
2. 验证 `50000/40000` 对当前实体夹爪的开合效果。
3. 验证紫色方块偏离时的真机低速视觉修正，以及居中后停止。
4. 完成搜寻、居中和轨迹验证后，进行一次 `GRASP_READY` 自动抓取测试。

## 推荐安全操作顺序

1. 清空机械臂工作区并确认急停/手动断电方式可用。
2. 只启动一套实体流程。
3. 先观察两个图像窗口和实体反馈，再开启运动门。
4. 开启运动门、arm 适配器后，才启动视觉搜索。
5. 发生异常时先停止视觉控制，再 disarm/关闭运动门，最后结束相关进程。

### 实体驱动使能命令

一键脚本按键 `1` 会执行下面的实体驱动使能；如果需要手动诊断，也可执行：

```bash
timeout 8s ros2 service call \
  /enable_srv \
  piper_msgs/srv/Enable \
  "{enable_request: true}"
```

必须返回 `enable_response: true`。手动启动时，完成实体使能后再调用适配器的 `enable_motion` 和 `arm`。一键脚本按 `1` 自动完成这三步并启动视觉流程；按 `2` 只停止视觉和 Servo，不调用 `disarm` 或 `/enable_srv false`；按 `0` 退出时才执行完整关闭。

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
1  一键使能、复位、自动搜寻、居中并抓取
2  紧急停止当前视觉/Servo 运动，但保持 PiPER 和适配器使能
0  退出并自动关闭运动门
```

### 4. 自动抓取前的最低验证

自动流程进入 `GRASP_READY` 前，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/piper_ros/install/setup.bash
source ~/piper_ws/install/setup.bash

timeout 8s ros2 run tf2_ros tf2_echo \
  gripper_base camera_color_optical_frame
```

必须持续输出变换，且不能出现 `two or more unconnected trees`。同时检测窗口应显示紫色方块，状态话题应为 `GRASP_READY`。如果需要立即停止当前运动，按 `2`；它不会失能 PiPER。
