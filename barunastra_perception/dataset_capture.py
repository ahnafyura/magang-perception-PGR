"""Menyimpan gambar kamera WAM-V untuk dataset YOLO.

Gerakkan kapal (teleop) ke berbagai posisi dan arah sambil node ini berjalan.
Gambar yang tersimpan lalu dilabeli sendiri (Roboflow, CVAT, atau Label Studio)
dan diekspor dalam format YOLOv8.

Jalankan:
    ros2 run barunastra_perception dataset_capture --ros-args -p out_dir:=$HOME/dataset_buoy

Hasil: out_dir/frame_000000.jpg, frame_000001.jpg, ...
"""
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from barunastra_perception.common import CAMERA_TOPIC, spin_until_ctrl_c


class DatasetCapture(Node):
    def __init__(self):
        super().__init__('dataset_capture')
        self.declare_parameter('out_dir', os.path.expanduser('~/dataset_buoy'))
        self.declare_parameter('period', 1.0)       # detik antar simpanan (waktu simulasi)
        self.declare_parameter('image_topic', CAMERA_TOPIC)
        self.out = os.path.expanduser(self.get_parameter('out_dir').value)
        os.makedirs(self.out, exist_ok=True)
        self.index = len([f for f in os.listdir(self.out) if f.endswith('.jpg')])
        self.bridge = CvBridge()
        self.last_save = None
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'menyimpan gambar ke {self.out}')

    def on_image(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_save is not None and t - self.last_save < float(self.get_parameter('period').value):
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.out, f'frame_{self.index:06d}.jpg'), frame)
        self.last_save = t
        self.index += 1
        if self.index % 20 == 0:
            self.get_logger().info(f'{self.index} gambar tersimpan')


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DatasetCapture()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
