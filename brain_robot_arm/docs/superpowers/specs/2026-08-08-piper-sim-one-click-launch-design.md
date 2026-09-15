# PiPER Gazebo 与 MoveIt2 一键启动脚本设计

## 目标

为当前 Ubuntu 22.04、ROS2 Humble 和 `~/piper_ros` 环境提供一个一键仿真入口。用户执行 `bash ~/start_piper_sim.sh` 后，脚本自动启动带夹爪的 Gazebo，等待控制器真正就绪，再启动 `piper_with_gripper_moveit`；脚本运行期间由同一个终端统一管理，用户按一次 `Ctrl+C` 即可关闭本次脚本创建的 MoveIt2、Gazebo 和相关子进程。

## 范围与边界

该脚本仅用于无实物的仿真开发，不配置 `can0`，不调用 `~/init_arm.sh`，不进入 Conda，也不启动真实机械臂的 `piper` 控制节点。脚本不会自动编译工作空间、安装系统依赖或修改官方 PiPER ROS2 源码。脚本只清理自己创建的进程，不主动终止用户在启动前已经运行的 Gazebo 或 MoveIt2 进程。

脚本的项目源文件保存在 Windows 共享项目的 `C:\ub\brain_robot_arm\scripts\start_piper_sim.sh`，在 Ubuntu 中对应 `/mnt/hgfs/ub/brain_robot_arm/scripts/start_piper_sim.sh`。部署时复制到 `~/start_piper_sim.sh` 并赋予执行权限，避免长期依赖 VMware 共享目录是否已挂载。

## 启动流程

1. 检查 `/opt/ros/humble/setup.bash`、`~/piper_ros/install/setup.bash` 和 `~/piper_ros/src/piper_sim/piper_gazebo/scripts/joint8_ctrl.py` 是否存在。
2. 检查夹爪镜像脚本是否具有执行权限；若没有，停止并给出可复制的 `chmod +x` 修复命令。
3. 检测当前用户会话中是否已有 PiPER Gazebo 或 PiPER MoveIt2 启动进程。若存在，脚本退出并提示用户先关闭旧实例，防止重复的控制器管理器、ROS节点和Gazebo服务相互冲突。
4. 加载 ROS2 Humble 和 `~/piper_ros/install/setup.bash`，设置 `LC_NUMERIC` 为使用小数点的区域值，全程使用 Ubuntu 系统 Python 3.10。
5. 在新的进程组中启动 `ros2 launch piper_gazebo piper_gazebo.launch.py`，保存该进程组标识。
6. 以短周期查询ROS图，最多等待60秒。只有机械臂、主夹爪和镜像夹爪的轨迹动作接口以及 `/joint_states` 话题都出现，才把Gazebo判定为就绪。该判据直接对应MoveIt2的运行依赖，不解析易受格式影响的控制器文字表格。
7. 在另一个新进程组中启动 `ros2 launch piper_with_gripper_moveit piper_moveit.launch.py`，保存该进程组标识。
8. 主脚本保持前台运行并监视两个启动进程。任一主进程异常退出时，脚本报告原因并清理另一组进程。

## 关闭与清理

脚本捕获 `Ctrl+C`、终止信号和正常退出事件。清理时先向本次启动的 MoveIt2 进程组发送中断信号，再向Gazebo进程组发送中断信号，并等待它们自行释放ROS节点和图形资源；若短时间内仍未退出，再对同一进程组依次发送终止信号和有时间上限的强制终止信号。清理范围以启动时记录的进程组标识为准，不能使用无范围的 `killall` 或 `pkill gazebo`，以免结束用户的其他仿真任务。

## 错误处理

- ROS2或工作空间未安装：指出缺失路径，脚本返回非零状态。
- 夹爪脚本不可执行：显示权限修复命令，不尝试静默修改官方源码。
- 旧实例仍在运行：拒绝重复启动，提示回到旧终端按 `Ctrl+C`。
- Gazebo在控制器就绪前退出：立即停止，不启动MoveIt2。
- 控制接口等待超过60秒：列出当前发现的动作接口和 `/joint_states` 状态，清理Gazebo并退出。
- MoveIt2提前退出：清理Gazebo，避免后台残留。

## 日志与用户提示

Gazebo和MoveIt2的标准输出保留在启动脚本所在终端，便于直接复制错误信息。脚本自身使用中文阶段提示，例如“正在启动Gazebo”“控制器已就绪”“正在启动MoveIt2”“按 Ctrl+C 统一关闭”。终端输出可能交错，但不额外引入日志服务；ROS2自身日志仍保存在 `~/.ros/log`。

## 验收标准

1. 未连接机械臂和CAN模块时，执行 `bash ~/start_piper_sim.sh` 能打开Gazebo和RViz2。
2. Gazebo中出现带夹爪的PiPER模型，三个轨迹动作接口和 `/joint_states` 话题均可发现。
3. MoveIt2仅在Gazebo控制器就绪后启动，RViz2显示 `MotionPlanning` 面板和机械臂模型。
4. 在脚本终端按一次 `Ctrl+C` 后，脚本启动的Gazebo、RViz2、MoveIt2和控制器节点全部退出。
5. 再次运行脚本能够正常启动，不因上一次运行残留进程而失败。
6. 脚本不访问 `can0`，不使能真实机械臂，也不调用现有的 `init_arm.sh`。

## 验证方法

先执行 Bash 语法检查，再在 Ubuntu 中进行一次冷启动、一次 `Ctrl+C` 统一关闭和一次关闭后的重新启动。运行期间用 `ros2 control list_controllers` 验证四个控制器状态，并用动作和话题列表验证MoveIt2所需接口；清理后确认不存在由本脚本创建的 `piper_gazebo.launch.py` 或 `piper_moveit.launch.py` 进程。真实机械臂不参与本脚本测试。
