# PiPER 腕部 Astra 未知空间禁止主动扫描：实施计划

关联设计：`docs/superpowers/specs/2026-09-21-wrist-camera-unknown-space-safe-active-scan-design.md`

## 实施原则

所有实体运动保持由既有 `piper_jog_adapter.py` 或既有真机轨迹发布路径输出；新代码不能绕过运动门、适配器、关节限位和急停话题。每个阶段独立构建、独立验证。Windows 仅做静态检查；Ubuntu/RViz 与真机验证由操作者执行。

## 阶段 0：先验证现有点云场景链路，不修改运动行为

涉及文件：

- `piper_ws/src/brain_robot_ball_pick/config/sensors_3d_real.yaml`
- `piper_ws/src/brain_robot_ball_pick/launch/cube_visual_search.launch.py`
- `piper_ws/src/brain_robot_pick_place/scripts/runtime_monitor.py`

工作：

1. 在运行时监视器增加只读的 `/brain_robot_vision/filtered_points`、相机到 `base_link` TF、Planning Scene 服务/话题状态显示。
2. 保留现有 PointCloudOctomapUpdater 参数；不因未实测而猜测滤波或分辨率值。
3. 给出 Ubuntu/RViz 验收命令：确认原始点云、过滤点云、TF 和 Planning Scene 同时存在。

成功标准：不使机械臂运动时，操作者可确认当前会话的点云、TF 和 MoveIt 场景是否可用；不可用时显示明确原因。

## 阶段 1：实现会话地图状态门

新增组件：`active_scan_supervisor`（C++ ROS 2 节点，归入 `brain_robot_pick_place`）。

输入：`/camera/depth/points`、`/piper_moveit_joint_states`、TF、MoveIt Planning Scene/Octomap。
输出：

- `/brain_robot_active_scan/state`：`WAITING_FOR_MAP`、`INITIAL_STILL_OBSERVE`、`VIEW_SELECTED`、`CORRIDOR_CERTIFIED`、`NO_CERTIFIED_VIEW`、`MAP_STALE`。
- `/brain_robot_active_scan/diagnostic`：点云年龄、TF 状态、地图状态、拒绝原因。
- 一个供视觉控制器调用的“是否允许主动扫描轨迹”服务；服务请求包含候选关节轨迹，响应包含允许/拒绝、失败采样点和 `UNKNOWN`/`OCCUPIED` 原因。

工作：

1. 只在接收到新鲜点云、关节状态和有效 `base_link <- camera` TF 后将状态设为可用。
2. 维护本进程内的会话时间戳与新鲜度；不写盘、不复用上次地图。
3. 初始阶段只等待连续稳定帧，不发布运动命令。
4. 先接入 MoveIt 已维护的 OctoMap；若运行时无法读取完整 Planning Scene/OctoMap，则只报告不可用，不退化为允许运动。

成功标准：断开点云、断开 TF 或地图过期时，状态稳定报告 `MAP_STALE`/不可用且所有候选路径被拒绝。

## 阶段 2：实现未知空间走廊门

涉及文件：

- 新增 `active_scan_supervisor.cpp`
- `piper_ws/src/brain_robot_pick_place/CMakeLists.txt`
- `piper_ws/src/brain_robot_pick_place/package.xml`

工作：

1. 将候选 `JointTrajectory` 按关节/空间步长插值成离散 RobotState。
2. 使用 MoveIt 的 RobotModel 和链接碰撞几何体，先以保守链接包络采样实现；采样半径加入配置的 `unknown_space_padding_m`。
3. 对每个采样状态查询当前 OctoMap：占据体素拒绝；未知体素也拒绝；仅明确空闲体素通过。
4. 保留 MoveIt 原有规划碰撞检查作为独立条件。未知空间门通过不等于普通碰撞检查通过，反之亦然；二者均通过才可执行。
5. 发布最早失败的轨迹点、关节状态、链接名和体素类别，便于监视器定位。

成功标准：构造含未知、占据、自由三类体素的离线/最小测试场景时，只有全自由走廊被授权；不以“无点”当作自由。

## 阶段 3：接入有限观察姿态状态机

涉及文件：

- `piper_ws/src/brain_robot_pick_place/src/visual_search_controller.cpp`
- `piper_ws/src/brain_robot_ball_pick/config/cube_task_real.yaml`
- `piper_ws/src/brain_robot_ball_pick/launch/cube_visual_search.launch.py`

工作：

1. 按键 `1` 的零点复位完成后，进入 `INITIAL_STILL_OBSERVE`，等待地图状态门允许，而不是立刻进入既有 PREPARE。
2. 在 YAML 中定义不超过 5 个观察候选关节姿态及低速比例；候选只描述相机朝向，不包含环境尺寸或障碍物坐标。
3. 观察姿态轨迹先由现有 MoveIt 规划，再调用未知空间走廊门；拒绝时跳过该候选，选择下一个。
4. 观察位到达后等待点云更新，再决定是否继续观察、进入现有 SEARCH/ALIGN，或报告 `ACTIVE_SCAN_NO_CERTIFIED_VIEW` 并保持原位。
5. 不改变目标丢失后的可信历史重捕获、颜色候选重获和持续 TRACK_HOLD 语义。

成功标准：按键 `1` 在地图未就绪时不动；只有被双重验证的观察位才低速执行；没有安全候选时不盲扫、不复位循环。

## 阶段 4：接入抓取微步路径

涉及文件：

- `piper_ws/src/brain_robot_pick_place/src/grasp_lift_executor.cpp`
- `piper_ws/src/brain_robot_ball_pick/config/cube_task_real.yaml`

工作：

1. 对预抓取轨迹和每个既有 8 mm `computeCartesianPath` 微步调用未知空间走廊门。
2. 路径被拒绝时不闭爪、不抬升，发布 `MICRO_APPROACH_UNKNOWN_CORRIDOR` 或 `MICRO_APPROACH_OCCUPIED_CORRIDOR`，并请求现有视觉重获。
3. 目标方块的局部接触策略单独实现：只允许明确的夹爪接触区域接近目标，不能关闭整张 OctoMap、关闭全局碰撞检查或允许其他连杆接触环境。

成功标准：任何微步经过未知或占据空间都会在执行前拒绝；点云/目标失效时仍使用现有深度门和反馈确认逻辑。

## 阶段 5：运行时诊断、安装和文档

涉及文件：

- `piper_ws/src/brain_robot_pick_place/scripts/runtime_monitor.py`
- `scripts/start_brain_robot_cube_real.sh`
- `scripts/install_piper_cube_pick.sh`
- `docs/purple_cube_real_arm_status.md`

工作：

1. 在现有单窗口监视器中显示主动扫描状态、点云/地图年龄、候选编号、走廊结果和拒绝原因；不创建额外高频终端日志。
2. 一键启动脚本启动新节点，日志继续沿用已实现的 20 MB 单文件、100 MB 总量和磁盘低水位限制。
3. 更新状态文档，记录源代码验证、Ubuntu/RViz 验收结果和未验证硬件限制。

成功标准：操作者只需一个监视终端，就能区分“目标未找到”“地图未就绪”“候选路径未知”“候选路径占据”“规划失败”和“微步抓取拒绝”。

## 验证顺序

1. Windows：C++ 结构检查、YAML 解析、Python 编译、`git diff --check`。
2. Ubuntu：受影响包 `colcon build --packages-select brain_robot_pick_place brain_robot_ball_pick`。
3. Ubuntu/RViz：阶段 0 的只读场景验证。
4. Ubuntu 真机：先空场，只开放第一观察位；再逐步增加观察位。
5. 最后才允许低速抓取微步验证；不得以笔记本电脑、硬质支架或人体作为碰撞测试物。

## 提交边界

每一阶段单独提交。只有在该阶段的静态检查通过后，才进入下一阶段；实机未验证的内容在状态文档中标为“待 Ubuntu/RViz/真机验收”，不得称为已验证。
