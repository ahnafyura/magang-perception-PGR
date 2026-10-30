"""Alat bantu mencari rentang HSV dengan slider (trackbar OpenCV).

Sumber gambar: topic kamera (default) atau file gambar.
    ros2 run barunastra_perception hsv_tuner
    ros2 run barunastra_perception hsv_tuner --ros-args -p image_file:=/path/gambar.png

Geser slider sampai hanya buoy yang putih di jendela "mask".
Tekan  p  untuk mencetak nilai, q  untuk keluar.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from barunastra_perception.common import CAMERA_TOPIC, spin_until_ctrl_c

WIN = 'hsv_tuner'
SLIDERS = [('H min', 0, 179), ('H max', 10, 179), ('S min', 120, 255),
           ('S max', 255, 255), ('V min', 70, 255), ('V max', 255, 255)]


class HsvTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.declare_parameter('image_file', '')
        self.declare_parameter('image_topic', CAMERA_TOPIC)
        self.bridge = CvBridge()
        self.frame = None
        path = self.get_parameter('image_file').value
        if path:
            self.frame = cv2.imread(path)
            if self.frame is None:
                raise RuntimeError(f'gambar tidak bisa dibaca: {path}')
        else:
            self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                     qos_profile_sensor_data)

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        for name, value, maximum in SLIDERS:
            cv2.createTrackbar(name, WIN, value, maximum, lambda _v: None)
        self.create_timer(1 / 30, self.update)

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def update(self):
        if self.frame is None:
            cv2.waitKey(1)
            return
        v = [cv2.getTrackbarPos(name, WIN) for name, _, _ in SLIDERS]
        low, high = (v[0], v[2], v[4]), (v[1], v[3], v[5])
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        hasil = cv2.bitwise_and(self.frame, self.frame, mask=mask)
        tampil = np.hstack([self.frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), hasil])
        cv2.imshow(WIN, cv2.resize(tampil, None, fx=0.5, fy=0.5))    # gambar 1280 px, diperkecil
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'):
            print(f'low={low} high={high}', flush=True)
        elif key == ord('q'):
            raise KeyboardInterrupt


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HsvTuner()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
