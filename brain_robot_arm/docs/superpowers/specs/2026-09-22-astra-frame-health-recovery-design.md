# Astra 实际图像帧健康恢复设计

## 背景

一键启动脚本目前通过 ROS 图检查 `/camera/camera` 和图像话题是否存在。Astra 节点会在 Orbbec SDK 成功打开设备前创建 Publisher，因此 USB 透传异常后可能出现“节点和话题存在、但没有任何图像帧”的状态。此时紫色方块检测器没有输入，`image_view` 没有调试图像，视觉控制器只能继续无目标搜索。

已观测到的底层证据包括 VMware 客户机 `xhci_hcd` 传输错误和 `libuvc.so` 崩溃。该设计不试图修复 VMware 或 USB 硬件；它只使一键流程在相机进程失去实际帧时自动重建 Astra SDK 会话。

## 目标与范围

脚本应按“真实图像帧”而不是 ROS 话题存在性判断相机是否工作。在连续无帧时，只终止并重启由脚本启动的 Astra 进程组。

以下组件必须保持运行且不接收停止、复位或失能请求：PiPER 驱动、运动适配器、MoveIt/Servo、视觉搜索控制器、检测器和 `image_view`。相机恢复后，检测器和搜索控制器应依靠现有 ROS 话题自动恢复工作。

不在本次范围内：修改 Orbbec SDK、VMware USB 控制器配置、改变机械臂搜索/对齐/抓取策略，或重新引入“启动后无帧即退出整个流程”的门禁。

## 设计

启动脚本把 Astra 的独立 `setsid` 进程组标识保存为相机进程组。现有日志维护后台任务旁新增相机健康后台任务。

健康任务以固定间隔检查 `/camera/color/image_raw` 的实际消息活动；只有在指定观察窗口内收到至少一帧才视为健康。ROS 图里存在 Publisher 或 Subscriber 不算健康。

连续无帧达到阈值后，任务记录一次 `CAMERA_NO_FRAMES` 日志，向已保存的 Astra 进程组发送终止信号，等待其退出，再以相同的 `ros2 launch astra_camera astra.launch.py` 命令和日志路径启动新的独立进程组。它不调用任何 PiPER、Servo、视觉控制器或抓取服务。

为避免 USB 持续断开时频繁重启，失败重启之间采用有限次数的递增等待；成功收到图像帧后重试计数清零，并记录 `CAMERA_FRAMES_RECOVERED`。达到连续失败上限后，任务保持存活并以较长周期继续探测，不退出一键流程，也不影响机械臂使能状态。

脚本退出时，现有清理路径必须同时终止相机健康任务和当前 Astra 进程组。

## 失败处理

如果 Astra 重启后仍无法打开 Orbbec 设备，日志应保留 Astra SDK 原始错误，例如 `Waiting for device connection...`。健康任务只说明“无帧并已重启/重试”，不将 VMware USB 传输故障错误归因于视觉控制器。

## 验证标准

1. 在相机正常时，健康任务不会重启 Astra，`/camera/color/image_raw` 保持有帧。
2. Astra 已存在但实际无帧时，脚本能检测到这一状态并只重启 Astra。
3. 相机重新出帧后，检测器与 `/image` 无需额外终端即可恢复。
4. 恢复过程不调用 `/enable_srv`、`/piper_jog_adapter/*`、`/visual_search_controller/*` 或 Servo 停止服务。
5. 连续无帧不会造成无限高频重启或日志/磁盘失控。
