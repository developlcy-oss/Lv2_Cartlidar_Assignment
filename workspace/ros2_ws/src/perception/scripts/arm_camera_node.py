#!/usr/bin/env python3
"""팔 카메라(Arducam UVC) 드라이버: OpenCV V4L2 로 MJPG 를 받아 풀어 Image · CameraInfo 를 발행한다.

usb_cam 0.8.1(lyrical)은 이 카메라의 1280×720 · 1920×1080 MJPG 에서 세그폴트로 죽고(Jetson 실물, #119),
YUYV 는 640×480 · 320×240 뿐인데 이 카메라의 4:3 모드는 세로 줄무늬가 생긴다 → 1280×720 MJPG 를 OpenCV 로 받는다.

bringup sensors.launch.py 가 띄운다. 따로 실행:
  ros2 run perception arm_camera_node.py --ros-args -r __ns:=/arm_camera -p device:=/dev/arm_cam
발행: image_raw (sensor_msgs/Image, bgr8) · camera_info (보정 파일이 없으면 크기만 채운 빈 값)
"""
import threading
import time

import cv2
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path, width, height):
    """ROS camera_info yaml(camera_calibration 출력 형식)을 읽는다. 없으면 크기만 채운 빈 값."""
    info = CameraInfo(width=width, height=height)
    if not path:
        return info
    with open(path.replace('file://', ''), encoding='utf-8') as f:
        c = yaml.safe_load(f)
    info.width, info.height = c['image_width'], c['image_height']
    info.distortion_model = c.get('distortion_model', 'plumb_bob')
    info.d = list(c['distortion_coefficients']['data'])
    info.k = list(c['camera_matrix']['data'])
    info.r = list(c['rectification_matrix']['data'])
    info.p = list(c['projection_matrix']['data'])
    return info


class ArmCameraNode(Node):
    def __init__(self):
        super().__init__('arm_camera')
        p = {
            'device': '/dev/arm_cam', 'width': 1280, 'height': 720, 'fps': 30.0,
            'frame_id': 'arm_camera_optical_frame', 'camera_info_url': '', 'reopen_s': 1.0,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        self.cfg = {k: self.get_parameter(k).value for k in p}
        # reliable 로 내도 best effort 구독(perception.yaml image_qos: sensor_data)과 연결된다
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_img = self.create_publisher(Image, 'image_raw', qos)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info', qos)
        self.info = load_camera_info(self.cfg['camera_info_url'], self.cfg['width'], self.cfg['height'])
        self.cap = None
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def open(self):
        c = self.cfg
        cap = cv2.VideoCapture(c['device'], cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, c['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c['height'])
        cap.set(cv2.CAP_PROP_FPS, c['fps'])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # 오래된 장을 쌓지 않음
        if not cap.isOpened():
            cap.release()
            return None
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"열림 {c['device']} {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps (MJPG)")
        if (w, h) != (c['width'], c['height']):
            self.get_logger().warning(f"요청 {c['width']}x{c['height']} 와 다름 — camera_info 크기를 실제로 맞춤")
            self.info.width, self.info.height = w, h
        return cap

    def loop(self):
        while self.running and rclpy.ok():
            if self.cap is None:
                self.cap = self.open()
                if self.cap is None:
                    self.get_logger().warning(f"{self.cfg['device']} 를 못 엶 — {self.cfg['reopen_s']} s 뒤 다시", throttle_duration_sec=5.0)
                    time.sleep(self.cfg['reopen_s'])
                    continue
            ok, frame = self.cap.read()
            stamp = self.get_clock().now().to_msg()
            if not ok:
                self.get_logger().warning('영상 읽기 실패 — 장치를 다시 엶', throttle_duration_sec=5.0)
                self.cap.release()
                self.cap = None
                continue
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, self.cfg['frame_id']
            msg.height, msg.width = frame.shape[:2]
            msg.encoding, msg.step = 'bgr8', frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.info.header = msg.header
            try:
                self.pub_img.publish(msg)
                self.pub_info.publish(self.info)
            except Exception:                  # 종료 중(컨텍스트가 먼저 닫힘)이면 조용히 끝냄
                if not rclpy.ok():
                    break
                raise

    def destroy_node(self):
        self.running = False
        try:
            self.thread.join(timeout=2.0)      # 읽기 스레드가 한 장 읽고 끝날 때까지
        except KeyboardInterrupt:
            pass
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                          # 종료 신호로 컨텍스트가 먼저 닫히면 spin 이 RCLError 를 낸다
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
