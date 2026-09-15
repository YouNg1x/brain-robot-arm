#!/usr/bin/env python3
import cv2
import numpy as np
from pyorbbecsdk import Pipeline, Config
def main():
    # 初始化相机管道
    pipeline = Pipeline()
    config = Config()
    # 启用彩色流、深度流
    config.enable_stream(0)   # COLOR
    config.enable_stream(1)   # DEPTH
    pipeline.start(config)
    print("相机已启动，按 ESC 键退出")

    while True:
        # 等待一帧数据（超时100毫秒）
        frames = pipeline.wait_for_frames(100)
        if frames is None:
            continue

        # ---- 彩色图像处理 ----
        color_frame = frames.get_color_frame()
        if color_frame is not None:
            color_data = np.asarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR)
            # 可以在彩色图上画一个中心点
            h, w = color_image.shape[:2]
            cv2.circle(color_image, (w//2, h//2), 5, (0,0,255), -1)
            cv2.imshow("Color", color_image)

        # ---- 深度图像处理 ----
        depth_frame = frames.get_depth_frame()
        if depth_frame is not None:
            depth_data = np.asarray(depth_frame.get_data(), dtype=np.uint16)
            # 归一化显示（0-255）
            depth_vis = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = np.uint8(depth_vis)
            cv2.imshow("Depth", depth_vis)

            # ---- 获取画面中心的深度值（毫米）----
            center_y, center_x = depth_data.shape[0]//2, depth_data.shape[1]//2
            depth_mm = depth_data[center_y, center_x]
            print(f"中心点深度: {depth_mm} mm")

        # 按 ESC 键退出
        if cv2.waitKey(1) == 27:   # 27 是 ESC 键
            break

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
