# PiPER 视觉发现后直接点位抓取设计

## 目标

视觉模块只负责在腕部 RGB-D 图像中发现红杯；它不再负责靠近、以深度阈值切换阶段，或用 Servo 强制多个关节完成抓取姿态。检测器输出的最大归一化图像轴误差小于 0.20 后，控制权从视觉 Servo 交给 MoveIt2。

对于 640 x 480 图像，发现门槛是 `abs(dx) <= 128 px` 且 `abs(dy) <= 96 px`。这个门槛只表示目标已被可靠发现，不表示杯子必须位于精确图像中心。

## 状态和职责

流程简化为：

`SEARCH -> TARGET_ACQUIRED -> POINT_GRASP -> CLOSE -> ATTACH -> LIFT -> HOLDING`

- `SEARCH`：保留现有视觉搜索和图像误差控制，用于把杯子带入发现门槛。
- `TARGET_ACQUIRED`：停止 MoveIt Servo，不再发送 J1/J5 或 J2/J3/J5 视觉关节速度。
- `POINT_GRASP`：抓取执行器读取最新 RGB 像素、有效深度和 `CameraInfo`，将杯心反投影为相机三维点；经 TF 转换到世界坐标后，生成夹爪世界水平的抓取 `PoseStamped`，交给 MoveIt2 一次规划和执行。
- `CLOSE -> ATTACH -> LIFT -> HOLDING`：沿用当前仿真闭爪、MoveIt 场景附着、Gazebo 杯子跟随和世界 `+Z` 抬升逻辑。

MoveIt2 负责整段轨迹内的全部关节协调：J2 增大、J3 减小、J5 跨过零位到负角度、J4/J6 姿态补偿，以及 J1 的必要转动。控制器不再假设这些关节在视觉阶段应遵循固定速度或固定中间角度。

## 深度和位姿

深度不再参与任何搜索、接近或阶段切换阈值。它仅用于在进入 `POINT_GRASP` 后构造三维抓取点：

`X = (u - cx) * Z / fx`

`Y = (v - cy) * Z / fy`

`Z = depth`

没有有效深度、相机内参或相机到夹爪 TF 时，不能构造 MoveIt2 点位目标，必须停止在未抓取状态并报告错误；这不是距离门槛，而是三维点计算的必要输入。

抓取目标以世界坐标系约束相机光轴在桌面平面内、画面下方对齐世界 `-Z`。通过已配置的相机到 `gripper_base` 固定外参得到夹爪目标位姿，因此夹爪在点位运动终点保持世界水平，而不是继承当前可能倾斜的 Servo 姿态。

## 参数和安全边界

- 新的 `target_acquire_error_ratio` 默认 `0.20`，由运行时图像宽高转换为像素门槛；不再把 640 x 480 写死在控制器中。
- 删除或停用 `stop_distance_m`、视觉接近 Twist、`GRASP_POSTURE` 的 J2/J3/J5 固定目标及相应深度门槛。
- 现有仿真配置允许自动点位抓取。
- 实体配置继续保持 `real_motion_enabled=false`、`simulation_only=false`、`auto_execute=false`。Gemini2 输入话题、内外参和安全停止验证后，才可单独解锁实体适配层。

## 验证

1. Gazebo 中杯子进入 20% 图像门槛后，观察视觉关节速度归零，状态进入 `TARGET_ACQUIRED` 后立刻进入 `POINT_GRASP`。
2. 终端确认点位抓取阶段没有 `depth <= 0.18 m` 的前进等待。
3. MoveIt2 轨迹中观察 J2/J3/J5 自动协调，J5 可从正角度过零到负角度，终点夹爪与桌面平行。
4. 只有规划和执行成功后才能闭爪、附着并沿世界 `+Z` 抬升至 `HOLDING`。
5. 单独记录 Gazebo 验证结果；它不证明 Gemini2、实体 PiPER、装水杯、SSVEP 或 EOG 已验证。
