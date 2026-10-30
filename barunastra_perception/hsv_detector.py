"""Deteksi buoy berdasarkan warna (HSV) dari kamera.

Subscribe : kamera depan kiri WAM-V      (sensor_msgs/Image, parameter image_topic)
Publish   : /perception/hsv/detections   (vision_msgs/Detection2DArray)
            /perception/hsv/debug        (sensor_msgs/Image, gambar + kotak)
            /perception/hsv/mask         (sensor_msgs/Image, mask gabungan)

Jalankan:
    ros2 run barunastra_perception hsv_detector
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from barunastra_perception.common import BOX_COLORS, CAMERA_TOPIC, make_detection, spin_until_ctrl_c

# Rentang HSV OpenCV: H 0-179, S 0-255, V 0-255.
# TODO 1: isi rentang (low, high) untuk tiap warna. Cari nilainya dengan hsv_tuner.
#         Petunjuk: merah ada di DUA ujung hue, jadi 'red' butuh dua pasang rentang.
#         Hati-hati: pohon di tepi danau juga hijau.
HSV_RANGES = {
    'red': [((0, 0, 0), (0, 0, 0))],
    'green': [((0, 0, 0), (0, 0, 0))],
}


class HsvDetector(Node):
    def __init__(self):
        super().__init__('hsv_detector')
        self.declare_parameter('min_area', 30.0)      # pixel^2, buang noise kecil
        self.declare_parameter('kernel_size', 5)      # ukuran kernel morphology
        self.declare_parameter('image_topic', CAMERA_TOPIC)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Detection2DArray, '/perception/hsv/detections', 10)
        self.pub_dbg = self.create_publisher(Image, '/perception/hsv/debug', 10)
        self.pub_mask = self.create_publisher(Image, '/perception/hsv/mask', 10)
        self.get_logger().info('hsv_detector siap, menunggu gambar kamera')

    def color_mask(self, hsv, color):
        """Mask biner (0 atau 255) untuk satu warna."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in HSV_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        return mask

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        k = int(self.get_parameter('kernel_size').value)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        min_area = float(self.get_parameter('min_area').value)

        out = Detection2DArray()
        out.header = msg.header
        debug = frame.copy()
        all_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for color in HSV_RANGES:
            mask = self.color_mask(hsv, color)
            # TODO 2: bersihkan mask dengan morphology memakai `kernel`:
            #         opening (hapus titik noise kecil) lalu closing (tutup lubang di objek).
            #         Fungsi: cv2.morphologyEx(mask, cv2.MORPH_..., kernel)
            all_mask |= mask

            # TODO 3: cari kontur di mask (cv2.findContours), buang yang luasnya
            #         < min_area (cv2.contourArea), buat kotak (cv2.boundingRect), lalu
            #         tambahkan ke out.detections dengan make_detection(msg.header, color,
            #         1.0, pusat_x, pusat_y, lebar, tinggi). Gambar kotaknya di `debug`
            #         dengan cv2.rectangle memakai warna BOX_COLORS[color].

        self.pub_det.publish(out)
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8', header=msg.header))
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(all_mask, encoding='mono8', header=msg.header))


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = HsvDetector()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
