# 按键 1 的零点复位后搜索设计

## 目标

实体 PiPER 一键脚本中的按键 `1` 必须从确定的六轴软件零点开始视觉任务，不能从机械臂当前姿态直接移动到观测姿态。

## 按键定义

- `1`：停止当前视觉控制；使能 PiPER 和保护适配器；执行六轴软件零点复位；确认关节反馈到 `[0, 0, 0, 0, 0, 0] rad`；再移动到观测姿态；最后开始搜索和居中。
- `2`：仅在视觉状态为 `GRASP_READY` 时请求抓取。
- `0`：停止当前视觉控制，执行六轴软件零点复位，并在零点停住；不开始观察、搜索或抓取。
- `Ctrl+C`：关闭视觉控制、保护适配器运动门和 PiPER 实体使能，然后退出脚本。

## 实现边界

零点复位沿用视觉控制器的 `/visual_search_controller/reset` 服务。该服务发布从当前反馈关节位置到 `reset_joint_positions` 的实体关节轨迹，等待反馈进入容差后发布 `RESET_COMPLETE`。这属于软件目标位置复位，不会修改编码器零点或调用厂商硬件 homing。

启动脚本的按键 `1` 将串行调用零点复位服务，并轮询视觉控制器的 `reason` 话题直到收到 `RESET_COMPLETE`；只有成功后才调用已有的 `start_visual_sequence`。零点复位失败或超时则不进入观测姿态和搜索。

## 验收

1. 按 `1` 时，状态首先为 `PREPARE`、原因是 `RESET_AUTHORIZED`，并且 `/joint_states` 最终接近六轴 `0 rad`。
2. 在 `RESET_COMPLETE` 前，不应出现观测姿态或搜索的轨迹命令。
3. `RESET_COMPLETE` 后，脚本才调用原有视觉启动服务，状态进入 `PREPARE`，随后进入 `SEARCH`/`ALIGN`。
4. 按 `0` 后，状态为 `RESET_COMPLETE` 并保持 `STOPPED`，不会自动进入 `PREPARE` 或 `SEARCH`。
