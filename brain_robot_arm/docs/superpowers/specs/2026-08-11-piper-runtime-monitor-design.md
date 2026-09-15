# PiPER 视觉抓取运行时监控终端设计

## 目标

提供一个独立、只读的 Ubuntu 终端监控工具，用于解释视觉搜索、视觉交接和 MoveIt2 抓取为什么停在某一阶段。`monitor_brain_robot_demo.sh` 只负责加载环境并启动常驻的 ROS2 Python 订阅节点；节点只订阅一次并定时渲染，避免通过反复启动 `ros2 topic echo` 丢失数据。工具不得发布关节命令、调用开始/停止服务、修改参数或改变 Gazebo/实体机械臂状态。

## 使用方式

一键演示启动后，在第二个终端执行：

```bash
bash ~/monitor_brain_robot_demo.sh
```

脚本每 0.5 秒刷新；按 `Ctrl+C` 只退出监控，不影响仿真主进程。

## 显示内容

- 节点存在性：视觉控制器、检测器、抓取执行器、MoveIt2。
- 视觉状态：`/brain_robot_visual_control/state`、`reason`、`search_direction`。
- 抓取状态：`/brain_robot_grasp/state`。
- 感知：`target_valid`、`depth_valid`、像素 `dx/dy`、归一化误差、相机坐标系三维点。
- 规划诊断：执行器计算的世界坐标系抓取目标位姿；若点位规划失败，同时显示失败时的当前末端位姿。
- IK 归因：若点位规划失败，显示无碰撞 IK 是否有解；`IK_OK_PLAN_FAILED` 表示姿态可达但路径或碰撞失败，`IK_ERROR_CODE_*` 表示目标姿态本身无解或被 MoveIt2 拒绝。
- 机械臂：`/joint_states` 中 J1--J6 的角度以及夹爪关节。
- Servo 状态，以及控制器实际加载的关键门槛。

## 状态门槛解释

1. `SEARCH` 或 `LOCAL_SEARCH`：等待连续 `target_acquire_frames` 个新鲜有效目标。
2. `ALIGN`：仅用图像误差控制搜索关节；连续 `align_stable_frames` 帧满足 `max(abs(dx)/width, abs(dy)/height) <= target_acquire_error_ratio`，进入 `TARGET_ACQUIRED`。
3. `TARGET_ACQUIRED`：视觉 Servo 停止；抓取执行器必须收到新鲜三维相机点、可用 TF、MoveIt2 规划服务和仿真场景杯子，之后依次显示开爪、点位规划、闭爪和抬升状态。

深度不再是视觉阶段转换条件；只用于把像素目标反投影成三维抓取点。

## 范围和抓取策略

本次增加只读监控，并将仿真抓取改为两段。第 1 段 MoveIt2 到达与杯子保持安全距离的世界水平预抓取位姿；第 2 段从该姿态低速、短距离、世界水平地沿相机前方执行笛卡尔直线接近，然后闭爪、附着和抬升。

预抓取距离 `pregrasp_standoff_m` 必须大于最终 `grasp_depth_m`。若笛卡尔路径没有达到 99.5%，执行器中止并恢复仿真场景，不退化为可能碰杯的普通点位规划。实体配置继续以 `simulation_only=false`、`auto_execute=false` 锁定；在完成相机外参、夹爪偏置、速度和急停验证之前，不允许启用自动执行。

相机深度点并不等于夹爪夹持中心。`grasp_height_offset_m` 是施加在世界 Z 方向的正向校准量：仿真初值为 `0.05 m`，用于使夹爪几何体避开桌面；实体初值为 `0.0 m`，必须经 Gemini2--夹爪实测标定后才可启用自动执行。

## 验证

- 静态：脚本 Bash 语法正确，安装脚本复制并赋予可执行权限。
- 仿真：主程序运行时，监控可显示当前状态、门槛、关节和视觉数据；其进程退出不得终止任一 ROS2 节点。
- 实体：不执行；脚本只能读取 ROS2 图和参数，实体自动执行仍由现有物理配置锁定。
