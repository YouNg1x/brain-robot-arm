# Gazebo 与 Gemini2 共用的 C++ 红杯视觉节点设计

## 目标与边界

本阶段把红杯识别从 Python/Tkinter 查看器中分离出来，建立一个 ROS 2 C++ 视觉节点。节点既能订阅 Gazebo 腕部相机，也能在以后通过修改话题参数订阅 Gemini2，不把具体相机型号写进识别算法。第一版只完成红杯检测、像素误差输出和调试图像发布，不向机械臂发送运动命令；机械臂自动居中、深度接近和抓取属于后续独立阶段。

当前 Tkinter 查看器只保留为原始图像诊断工具。正式标注画面由 C++ 节点发布，并使用 ROS 2 的 `image_view` 显示，从而避免 Tkinter 每帧替换图像时出现的黑色横带。

## 方案选择

视觉节点使用 `rclcpp + cv_bridge + OpenCV`，通过标准 `sensor_msgs/msg/Image` 话题收发图像。相比继续使用纯 Python 字节循环，这种方案处理轮廓、形态学滤波和深度图更直接，运行开销更低；相比 Python OpenCV，它不依赖当前冲突的 NumPy ABI。仿真与实物共用同一算法文件，差异只存在于启动文件中的输入话题参数。

## 节点与接口

节点名为 `cup_detector`，默认订阅以下参数化输入：

- `rgb_topic`：默认 `/wrist_camera/wrist_camera/image_raw`。
- `depth_topic`：默认 `/wrist_camera/wrist_camera/depth/image_raw`，第一版允许关闭深度处理。
- `camera_info_topic`：默认 `/wrist_camera/wrist_camera/camera_info`。
- `use_depth`：第一版默认 `false`，接入 Gemini2 或验证仿真深度后设为 `true`。

节点发布：

- `/brain_robot_vision/debug_image`，类型为 `sensor_msgs/msg/Image`，显示绿色画面中心、黄色红杯框、杯子中心和数值状态。
- `/brain_robot_vision/target_pixel`，类型为 `geometry_msgs/msg/PointStamped`；`x=u`、`y=v`、`z=depth_m`。未启用或没有有效深度时，`z` 为 `NaN`。
- `/brain_robot_vision/pixel_error`，类型为 `geometry_msgs/msg/Vector3Stamped`；`x=dx=u-cx`、`y=dy=v-cy`、`z=目标面积占画面比例`。
- `/brain_robot_vision/target_valid`，类型为 `std_msgs/msg/Bool`。没有满足条件的红色轮廓时发布 `false`，控制节点不能继续沿用旧目标。

所有输出沿用输入图像时间戳和相机光学坐标系。后续视觉伺服只读取这些输出，不直接依赖 OpenCV或相机驱动。

## 红杯检测流程

彩色图像经 `cv_bridge` 转换为 BGR，再转到 HSV。红色跨越 HSV 色相首尾，因此合并低色相和高色相两个掩膜；随后执行一次开运算去除散点、一次闭运算填补杯体内部的小孔。对所有外轮廓按面积过滤，选取面积最大的有效轮廓作为当前红杯，计算外接矩形、轮廓矩和中心点。

检测器使用参数保存色相、饱和度、亮度和最小面积阈值，默认值以当前 Gazebo 红杯为准，不能把阈值硬编码进控制节点。如果外接框接触画面边缘，调试图像标记 `CLIPPED`；它仍可用于转向居中，但后续不允许直接触发抓取。连续若干帧丢失目标时持续发布 `target_valid=false`。

## 显示与性能

调试图像以相机原分辨率发布，由以下标准命令显示：

```bash
ros2 run image_view image_view --ros-args \
  -r image:=/brain_robot_vision/debug_image
```

识别回调不执行轨迹规划或机械臂控制，只完成一次图像处理和消息发布。订阅队列深度设为 1，以实时处理最新画面并允许在虚拟机算力不足时丢弃旧帧，避免延迟不断累积。第一阶段接受 Gazebo 中约 3 FPS，但输出时间戳必须持续更新；实体 Gemini2 阶段再测量真实帧率和CPU占用。

## 仿真与实物复用

Gazebo 启动文件直接使用默认话题。Gemini2 接入后新增一个实物启动文件，将 `rgb_topic`、`depth_topic` 和 `camera_info_topic` 指向 Gemini2 ROS 2 驱动实际发布的话题。若实体相机输出颜色格式、深度单位或彩色/深度对齐方式不同，只在相机适配参数和深度转换层处理，不修改红杯轮廓检测和像素误差定义。

相机安装在机械臂腕部后必须完成相机内参验证以及相机光学坐标系到 `gripper_base` 的外参标定。仿真中固定关节的位姿不能直接当作实体 Gemini2 的标定结果。

## 与 SSVEP 和 EOG 的关系

视觉节点只负责报告目标位置，不负责决定抓取哪个物体。后续 SSVEP 模块产生高层目标选择，EOG提供确认、取消或急停信号；任务管理节点在收到合法选择与确认后启用对应目标的视觉跟踪。任何脑电指令都不能绕过目标有效性、图像超时、机械臂限位和急停条件。

## 验收标准

1. C++ 节点在 ROS 2 Humble 下完成编译，启动时不导入 Python NumPy 或 OpenCV。
2. Gazebo 红杯进入画面后出现稳定黄色框和中心点；桌面、机械臂和背景不应被识别为红杯。
3. 杯子位于画面右侧时 `dx>0`，左侧时 `dx<0`；位于下方时 `dy>0`，上方时 `dy<0`。
4. 杯子离开画面后 `target_valid` 变为 `false`，不能持续发布旧位置作为有效目标。
5. 标准 `image_view` 显示调试图像时不出现 Tkinter 黑色刷新横带。
6. 通过修改输入话题即可接入 Gemini2；实体识别、深度精度和机械臂闭环必须单独验证，不能由 Gazebo 结果代替。

## 实施顺序

先创建 C++ 红杯检测节点和参数文件，完成编译与静态检查；随后在 Gazebo 中验证识别框、误差符号、目标丢失和调试图像。确认检测稳定后再加入深度中值读取，最后另建视觉居中控制节点。每一步只增加一个闭环变量，第一版视觉检测绝不直接驱动真实机械臂。
