import cv2
import numpy as np
import threading
import pyrealsense2 as rs


class ArucoDetector:
    def __init__(self):
        self.exit = False  # 设置停止标志位
        self.aruco_frame = None
        self.aruco_distance = 0.0
        self.lock = threading.Lock()  # 用于线程安全
        self.start_detect()

    def calculate_distance(self, center1, center2):
        # 计算两个中心点之间的欧几里得距离
        return np.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)

    def detect_aruco_realsense(self):
        # 配置 RealSense 摄像头
        pipeline = rs.pipeline()
        config = rs.config()

        # 配置 RealSense 流：彩色流和深度流
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 启动摄像头
        pipeline.start(config)

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        arucoParams = cv2.aruco.DetectorParameters()

        while not self.exit:
            # 捕获 RealSense 帧
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 将帧转为 OpenCV 格式
            frame = np.asanyarray(color_frame.get_data())

            # 检测 ArUco 标记
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

            centers = []

            if len(corners) > 0:
                ids = ids.flatten()

                for (markerCorner, markerID) in zip(corners, ids):
                    # 提取标记的四个角
                    corners_ = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners_

                    # 将每个 (x, y) 坐标对转换为整数
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # 计算中心点 (x, y) 坐标
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    centers.append((cX, cY))

                    # 绘制 ArUco 标记的边界框
                    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                    # 绘制 ArUco 标记的中心点
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                    # 在图像上绘制 ArUco 标记 ID
                    cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 如果检测到两个 ArUco 标记，计算它们的中心点距离
                if len(centers) == 2:
                    distance = self.calculate_distance(centers[0], centers[1])
                    with self.lock:
                        self.aruco_distance = float(distance)

                elif len(centers) == 1:
                    with self.lock:
                        self.aruco_distance = 0.0
                    # print("Only one marker detected.")
                elif len(centers) == 0:
                    with self.lock:
                        self.aruco_distance = 0.0
                    # print("No markers detected.")

            with self.lock:
                self.aruco_frame = frame

    def get_aruco_img(self):
        with self.lock:
            return self.aruco_frame

    def get_aruco_pixel_distance(self):
        with self.lock:
            return self.aruco_distance

    def start_detect(self):
        self.thread = threading.Thread(target=self.detect_aruco_realsense)
        self.thread.start()

    def stop_detect(self):
        self.exit = True  # 设置停止标志位
        self.thread.join()

# 如果需要单独测试，可以添加如下内容
if __name__ == '__main__':
    detector = ArucoDetector()
    try:
        while True:
            frame = detector.get_aruco_img()
            distance = detector.get_aruco_pixel_distance()

            if frame is not None:
                cv2.imshow('Aruco Detection', frame)

            print(f"Distance between markers: {distance:.2f} pixels")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop_detect()
        cv2.destroyAllWindows()
