# PiPER 固定桌面杯子自动取放仿真设计

## 目标

在已经验证正常的 PiPER Gazebo、MoveIt2 和夹爪控制链路上，实现一个固定场景的自动取杯演示。场景包含低护理桌、标准圆柱杯和桌面安全交接区；机械臂从桌面侧向抓取杯子，将其抬起并放到交接区。桌子和杯子必须同时存在于 Gazebo 物理场景与 MoveIt 规划场景中，杯子夹住后必须在 Gazebo 和 RViz2 中同步跟随机械臂。

本阶段只验证仿真执行层，不接入 Gemini2、SSVEP、EOG、CAN 或真实机械臂。后续 Gemini2 只替换固定杯子坐标来源，SSVEP 和 EOG 只负责目标选择与执行确认，不直接操作关节。

## 工作空间边界

官方工作空间 `~/piper_ros` 作为只读底层依赖，不修改其中的驱动、URDF、Gazebo 和 MoveIt 配置。自定义功能包命名为 `brain_robot_pick_place`，Ubuntu 源码位置为 `~/piper_ws/src/brain_robot_pick_place`，Windows 主副本位置为 `C:\ub\brain_robot_arm\piper_ws\src\brain_robot_pick_place`。

构建自定义工作空间时先加载 `/opt/ros/humble/setup.bash` 和 `~/piper_ros/install/setup.bash`，再在 `~/piper_ws` 中执行 `colcon build --symlink-install`。运行时依次加载 ROS2、官方工作空间和自定义工作空间。

## 功能包组成

### 场景模型

- `models/care_table/model.sdf`：静态护理桌，带可视模型和碰撞模型。
- `models/medicine_cup/model.sdf`：圆柱杯，保留可视模型和碰撞模型，关闭重力以获得可重复的取放效果。
- 杯子不是液体仿真，不模拟杯把、液面、倾倒或滑动。

### 参数文件

`config/pick_place.yaml` 统一保存桌子尺寸、桌面高度、杯子尺寸、杯子起始位置、交接位置、预抓取距离、抬升高度、速度、加速度、规划时间、重试次数和各阶段停止参数。参数 `simulation_only` 固定默认为 `true`；第一版若被设置为 `false`，节点拒绝启动。坐标和尺寸不能散落在源代码中。

### scene_manager_node

负责检查 Gazebo 与 MoveIt 服务是否可用，清理同名旧模型，在 Gazebo 中生成 `care_table` 和 `medicine_cup`，并向 MoveIt 规划场景加入尺寸和位姿一致的桌子盒体与杯子圆柱体。节点提供场景重置服务，重复运行时不会产生同名模型冲突。

### cup_follow_node

通过 `std_srvs/srv/SetBool` 提供杯子跟随开关。开启跟随时，节点查询杯子当前 Gazebo 位姿以及 `world → gripper_base` 的 TF，计算并保存杯子相对夹爪的变换；之后以约 30 Hz 调用 Gazebo 实体状态服务更新杯子位姿。关闭跟随后，杯子停留在最后的放置位置。节点只移动名为 `medicine_cup` 的模型，不操作机械臂。

### pick_place_node

使用 MoveIt2 C++ `MoveGroupInterface` 创建 `arm` 和 `gripper` 两个规划组，执行抓取状态机。机械臂速度和加速度默认限制为 0.10，规划时间为 5 秒，每一步最多规划两次。节点通过规划场景接口附着和解除杯子，通过跟随服务同步 Gazebo 杯子。

### 启动文件

`scene.launch.py` 只启动场景管理与杯子跟随组件，不自动移动机械臂。用户确认桌子和杯子位置后，再单独运行 `pick_place_node`。第一版不把自动抓取合并进现有 `start_piper_sim.sh`，防止启动仿真时机械臂立即运动。

## 初始场景参数

所有坐标使用 `world` 米制坐标系，以下数值作为第一轮仿真的可调起点：

- 桌面尺寸：长 0.70 m、宽 0.50 m、厚 0.05 m。
- 桌面中心：`x=0.40, y=0.00, z=0.225`，因此桌面上表面高度为 0.25 m。
- 杯子尺寸：直径 0.07 m、高 0.10 m。
- 杯子初始中心：`x=0.40, y=-0.12, z=0.30`。
- 交接区杯子中心：`x=0.35, y=0.16, z=0.30`。
- 预抓取距离：沿末端接近轴后退 0.10 m。
- 垂直抬升高度：0.10 m。
- 放置上方安全高度：0.10 m。

机械臂执行前先规划到 SRDF 中已有的 `zero` 状态，再读取 `link6` 当前姿态作为侧向抓取的基础方向。后续预抓取、抓取、抬升和放置保持该末端方向，避免在第一版中依赖未经验证的硬编码四元数。若初始坐标不可达，只在 YAML 中逐项调整，不修改运动算法。

## MoveIt 与 Gazebo 场景同步

Gazebo 中的桌子和杯子用于可视化与仿真实体状态；MoveIt 中的桌子盒体和杯子圆柱体用于路径规划和碰撞检查。两份对象共享同一个 YAML 参数源，名称和位姿保持一致。

最终靠近杯子时，只允许 `gripper_base`、`link7` 和 `link8` 与 `medicine_cup` 接触，机械臂其他连杆仍必须避碰。夹爪闭合后，MoveIt 将杯子作为附着碰撞体连接到 `gripper_base`，触碰连杆为两侧夹爪；同时开启 Gazebo 杯子跟随。放置时先完成下降，再张开夹爪、解除 MoveIt 附着并关闭 Gazebo 跟随，最后恢复杯子为世界碰撞物体。

## 自动取放状态机

1. `SETUP`：等待 MoveIt、Gazebo、TF 和跟随服务，验证场景对象存在。
2. `HOME`：机械臂移动到 `zero` 状态并记录末端基础方向。
3. `OPEN`：夹爪移动到 SRDF 的 `open` 状态。
4. `PREGRASP`：规划到杯子侧面的预抓取点。
5. `APPROACH`：使用笛卡尔直线路径靠近杯身。
6. `CLOSE`：夹爪移动到 `close` 状态。
7. `ATTACH`：MoveIt 附着杯子并开启 Gazebo 跟随。
8. `LIFT`：使用笛卡尔直线路径垂直抬升 0.10 m。
9. `TRANSFER`：规划到交接区上方安全位置。
10. `LOWER`：使用笛卡尔直线路径下降到桌面放置高度。
11. `RELEASE`：张开夹爪、解除附着、关闭跟随并恢复世界碰撞体。
12. `RETREAT`：沿接近方向后退到安全距离。
13. `DONE`：打印执行完成和各阶段耗时，不自动循环。

## 分阶段运行

节点提供字符串参数 `stop_after`，合法值为 `pregrasp`、`approach`、`grasp`、`lift`、`transfer`、`place` 和 `done`。达到指定阶段后正常退出，不执行后续动作。缺省值为 `done`。

分阶段测试顺序固定为：只生成场景、运行到预抓取、运行到靠近、验证闭合与双场景附着、验证抬升、验证搬运、执行完整取放。不能在前一阶段尚未通过时直接跳到完整运行。

## 错误处理与安全状态

- 依赖服务、动作或 TF 在超时时间内不可用：不发送运动命令并退出。
- Gazebo 或 MoveIt 场景对象缺失：停止执行并提示先运行 `scene.launch.py`。
- 规划失败：清除旧目标后重试一次；再次失败则停在当前位置。
- 抓取前失败：保持夹爪张开，不开启杯子跟随。
- 附着后失败：保持夹爪闭合、杯子附着和跟随，不自动释放；用户检查后重置场景。
- 笛卡尔路径完成比例低于 0.95：视为失败，不执行残缺轨迹。
- 服务调用失败或 Gazebo 杯子跟随节点退出：停止后续运动。
- `simulation_only` 不是 `true`：拒绝启动，防止误用于真实机械臂。
- 任何错误均返回非零退出码，并打印失败阶段、MoveIt 错误码和建议检查项。

## 使用流程

1. 运行现有 `bash ~/start_piper_sim.sh`，等待 Gazebo、MoveIt2 和 RViz2 正常出现。
2. 加载 `~/piper_ws/install/setup.bash`，运行 `scene.launch.py`。
3. 在 Gazebo 和 RViz2 中检查桌子、杯子及碰撞场景，不手动移动对象。
4. 使用 `stop_after:=pregrasp` 开始逐阶段验证。
5. 所有阶段通过后，使用缺省 `stop_after:=done` 完成自动取放。

## 验收标准

1. Gazebo 和 RViz2 中都出现尺寸、位置一致的桌子与杯子。
2. MoveIt 规划轨迹不穿过桌面，机械臂非夹爪连杆不与杯子相交。
3. 夹爪闭合后，杯子在 Gazebo 和 RViz2 中都随夹爪抬升与搬运。
4. 杯子最终落在交接区桌面上，夹爪松开后杯子不再跟随。
5. `stop_after` 的每个阶段均能独立停止并给出明确状态。
6. 规划失败或服务异常时不会跨过失败阶段继续运动。
7. 整个功能包不访问 `can0`、不调用 `init_arm.sh`、不启动真实 PiPER 控制节点。
8. 关闭后重新启动场景不会因同名 Gazebo 或 MoveIt 对象而失败。

## 后续接口

Gemini2 接入后输出 `world` 或机械臂基座坐标系下的目标杯子位姿，替换 `cup.initial_pose` 参数。SSVEP 输出目标类别或操作选择，EOG 输出确认、取消和急停意图；只有目标有效且 EOG 确认后才触发 `pick_place_node`。真实机械臂阶段复用 MoveIt 状态机，但移除 Gazebo 场景管理与杯子跟随节点，并重新标定桌面、杯子和交接区坐标。
