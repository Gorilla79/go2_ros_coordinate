import os
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class Go2Publisher(Node):
    def __init__(self):
        super().__init__('go2_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'go2_coordinate', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # YOLO 모델 초기화
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model_human = YOLO("yolov8n.pt").to(device)
        self.model_pose = YOLO("yolov8n-pose.pt").to(device)

        # RealSense 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)

        self.get_logger().info('✅ Go2 Publisher Node Started')

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return

            # 이미지를 numpy 배열로 변환
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 이미지 중심 좌표 계산
            height, width = color_image.shape[:2]
            cx, cy = width // 2, height // 2

            # YOLO 사람 탐지 실행
            results_human = self.model_human.track(color_image, classes=[0], persist=True)

            boxes_human = []
            ids_human = []
            if results_human[0].boxes:
                human_boxes = results_human[0].boxes
                human_confidences = human_boxes.conf.cpu().numpy()
                boxes_human = human_boxes.xyxy.cpu().numpy()[human_confidences >= 0.6]
                ids_human = (
                    human_boxes.id.cpu().numpy()[human_confidences >= 0.6]
                    if human_boxes.id is not None
                    else []
                )

            coordinates = []

            # 탐지된 객체에 대해 좌표 변환 및 저장
            for box, human_id in zip(boxes_human, ids_human):
                x1, y1, x2, y2 = map(int, box)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                depth = depth_frame.get_distance(center_x, center_y)

                # ID를 int로 변환 (문제 해결 부분)
                human_id = int(human_id)  

                # 중심 기준 좌표 계산
                relative_x = center_x - cx
                relative_y = int(depth * 100)  # 깊이 값을 cm 단위로 변환

                # 좌표값 저장
                coordinates.extend([human_id, relative_x, relative_y])

                # 디버깅 출력
                self.get_logger().info(f"ID:{human_id}, X:{relative_x}, Y:{relative_y}")

            # ROS 메시지로 변환 및 발행
            if coordinates:
                msg = Int32MultiArray()
                msg.data = coordinates  # int 값으로 전달
                self.publisher_.publish(msg)
                self.get_logger().info(f"📢 Publishing coordinates: {coordinates}")

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

    def destroy(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
