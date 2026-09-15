# 脑控机械臂视觉仿真一键启动设计

## 目标

用户以后只执行 `bash ~/start_brain_robot_demo.sh`，脚本自动启动 Gazebo、PiPER带夹爪模型、MoveIt2/RViz2、桌子和红杯场景、C++红杯检测节点以及标准图像查看窗口。启动完成后只保留一个总控终端；在该终端按一次 `Ctrl+C`，统一关闭本次脚本启动的全部进程。

本脚本严格用于仿真，不初始化 CAN、不调用 PiPER实体驱动、不读取头环，也不自动执行抓杯轨迹。实体 PiPER、Gemini2和 SSVEP＋EOG 后续使用单独且名称醒目的真实设备启动入口，视觉检测核心节点继续共用。

## 方案

保留已经验证稳定的 `start_piper_sim.sh` 作为底层 Gazebo＋MoveIt2启动器，在其外层增加 `start_brain_robot_demo.sh`。外层脚本负责启动并监控底层脚本，等待 ROS控制接口和 MoveIt2就绪，再依次启动场景、红杯检测器和 `image_view`。这种包装方式不重写已有控制器等待逻辑，出现问题时也能单独运行底层脚本诊断。

原始 Tkinter腕部相机窗口默认不再自动启动，避免黑色刷新横带和重复窗口；它仍保留为手动原始图像诊断工具。正式演示画面只显示 `/brain_robot_vision/debug_image`。

## 启动与就绪检查

脚本按以下顺序工作：

1. 检查 ROS 2 Humble、`~/piper_ros`、`~/piper_ws`、底层启动脚本以及 `image_view` 是否存在，并拒绝在 Conda 环境中启动。
2. 启动 `start_piper_sim.sh`，等待 `/move_group` 和腕部彩色图话题出现。
3. 启动 `scene.launch.py`，等待 `/brain_robot_pick_place/scene_ready` 发布 `true`。
4. 启动 `cup_detector.launch.py`，等待 `/brain_robot_vision/debug_image` 出现发布者。
5. 启动标准 `image_view`，显示带中心十字、识别框和 `dx/dy` 的画面。

每一步都有超时和明确中文错误。任何关键进程提前退出时，总控脚本停止其他本次启动的进程，不留下重复 Gazebo、MoveIt2或识别节点。

## 进程与关闭策略

底层仿真、场景、检测器和图像查看器分别运行在独立进程组中。总控脚本只记录并终止自己创建的进程组，不使用宽泛的 `pkill`，因此不会误关用户其他终端中的无关 ROS程序。关闭时先停止图像查看器、检测器和场景，再停止底层仿真脚本；底层脚本继续负责安全关闭 MoveIt2和 Gazebo。

## 输出与验收标准

启动成功后终端显示“视觉仿真演示已就绪”，Gazebo、RViz2和一幅红杯检测画面均存在。红杯检测窗口不再出现 Tkinter 黑色横带，红杯应出现黄色框且当前右下位置满足 `dx>0、dy>0`。按一次 `Ctrl+C` 后，脚本启动的 Gazebo、RViz2、场景、检测器和图像查看器全部退出；再次执行同一条命令能够正常重新启动。

首次部署仍需同步代码和编译一次；完成后日常使用不再要求用户分别打开四个终端或重复输入环境加载命令。
