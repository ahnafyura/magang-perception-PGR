"""Deteksi buoy dengan model YOLO (Ultralytics) sebagai node ROS 2.

Subscribe : kamera depan kiri WAM-V       (sensor_msgs/Image, parameter image_topic)
Publish   : /perception/yolo/detections   (vision_msgs/Detection2DArray)
            /perception/yolo/debug        (sensor_msgs/Image)

Jalankan:
    ros2 run barunastra_perception yolo_detector
    ros2 run barunastra_perception yolo_detector --ros-args -p model:=/path/best.pt -p conf:=0.5
"""
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2DArray

from barunastra_perception.common import CAMERA_TOPIC, make_detection, spin_until_ctrl_c


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        default_model = os.path.join(
            get_package_share_directory('barunastra_perception'), 'models', 'buoy_vrx_yolov8n.pt')
        self.declare_parameter('model', default_model)
        self.declare_parameter('conf', 0.5)       # buang deteksi dengan skor di bawah ini
        self.declare_parameter('device', '')      # '' = otomatis, 'cpu', atau '0' (GPU)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('image_topic', CAMERA_TOPIC)

        path = self.get_parameter('model').value
        self.model = YOLO(path)
        self.get_logger().info(f'model {path}, kelas {self.model.names}')
        self.bridge = CvBridge()
        self.t_log = time.time()
        self.n_frames = 0

        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Detection2DArray, '/perception/yolo/detections', 10)
        self.pub_dbg = self.create_publisher(Image, '/perception/yolo/debug', 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        device = self.get_parameter('device').value or None
        result = self.model.predict(
            frame, conf=float(self.get_parameter('conf').value),
            imgsz=int(self.get_parameter('imgsz').value), device=device, verbose=False)[0]

        out = Detection2DArray()
        out.header = msg.header
        # TODO 4: ubah hasil YOLO menjadi Detection2D lalu masukkan ke out.detections.
        #   result.boxes.xywh -> tiap baris (pusat x, pusat y, lebar, tinggi) dalam pixel
        #   result.boxes.cls  -> id kelas, nama kelasnya self.model.names[int(id)]
        #   result.boxes.conf -> skor keyakinan
        #   Pakai .tolist() supaya jadi list Python, lalu make_detection(...).

        self.pub_det.publish(out)
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8', header=msg.header))

        self.n_frames += 1
        now = time.time()
        if now - self.t_log > 5.0:
            fps = self.n_frames / (now - self.t_log)
            self.get_logger().info(f'{fps:.1f} FPS, {len(out.detections)} objek di frame terakhir')
            self.t_log, self.n_frames = now, 0


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = YoloDetector()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
