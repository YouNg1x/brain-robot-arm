# PiPER 真机碰撞检查搜索设计

## 目标

在紫色方块视觉搜索阶段，不再直接连续发送未经规划的 J1/J5 速度扫描。每个短搜索段必须先基于当前关节反馈和 MoveIt Planning Scene 完成可达性、关节限位、自碰撞和环境碰撞检查；只有规划成功的短轨迹才允许交给实体适配器执行。目标检测成功后立即取消未执行的搜索段，转入现有视觉对齐状态。

## 现有条件

- RGB-D 点云通过 `PointCloudOctomapUpdater` 进入 MoveIt。
- `/brain_robot_vision/filtered_points` 已由 `move_group` 发布，实测约 1 Hz。
- `/brain_robot_active_scan/diagnostic` 已能进入 `MAP_INPUT_READY`。
- `/piper_jog_adapter` 是实体 PiPER 唯一的 `/joint_commands` 输出路径。
- 当前 `visual_search_controller` 的连续 J1/J5 JointJog 搜索必须被替换或封装，不能与分段轨迹同时输出。

## 推荐架构

`visual_search_controller` 保留目标状态机和搜索边界，但搜索动作改为短段规划请求：

1. 读取 `/piper_moveit_joint_states`，以当前反馈作为规划起点。
2. 根据 J1/J5 搜索方向生成一个小步长候选关节状态，J2/J3/J4/J6 保持当前反馈。
3. 通过 MoveIt `MoveGroupInterface` 规划到候选状态；规划器负责关节限制、自碰撞和 Planning Scene 环境碰撞。
4. 将成功轨迹发布到现有 `/brain_robot_grasp/arm_trajectory`，由 `piper_jog_adapter` 限速执行。
5. 轨迹执行期间不发布 JointJog；收到 `target_valid=true`、目标丢失、急停或规划场景过期时取消剩余搜索轨迹。
6. 轨迹完成后重新读取点云和关节状态，再生成下一段。

`active_scan_supervisor` 继续只负责点云证据和诊断，不直接发布机械臂命令。

## 失败处理

- IK 或规划失败：记录候选关节目标和失败原因，跳过该方向，尝试下一个边界内候选。
- Planning Scene 或过滤点云过期：停止搜索并发布明确诊断，不发送非零命令。
- 所有候选均失败：保持当前位置，等待新的 `1` 授权，不自动突破边界。
- 目标出现：停止搜索轨迹，只有完整 `target_valid`（紫色、正方体形状、尺寸、深度和连续帧）确认后才进入 `ALIGN`。

## 验收标准

1. 无目标时，搜索每一段都有对应的规划请求和成功/失败诊断。
2. 任何搜索段都不会越过 J1/J5 软件范围，且不会在 Planning Scene 碰撞时执行。
3. `/joint_commands` 只来自适配器，搜索期间不再同时存在 JointJog 与搜索轨迹输出。
4. 目标出现后，未执行的搜索段被停止并进入 `ALIGN`；仅颜色候选不能触发对齐。
5. 点云或 Planning Scene 过期时，机械臂保持当前反馈位置。

## 暂不实现

- 不让机械臂主动扫描未知空间并永久保存地图。
- 不绕过 MoveIt 直接发送直线抓取或未经规划的关节目标。
- 不把点云体素监督器变成第二个实体命令发布者。
