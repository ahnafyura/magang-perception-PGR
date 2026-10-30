"""Nama topic VRX dan fungsi bantu yang dipakai beberapa node."""
import signal

import rclpy
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

# topic sensor WAM-V di VRX
CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera_sensor/image_raw'
CAMERA_INFO_TOPIC = '/wamv/sensors/cameras/front_left_camera_sensor/camera_info'
CLOUD_TOPIC = '/wamv/sensors/lidars/lidar_wamv_sensor/points'
BASE_FRAME = 'wamv/wamv/base_link'

# urutan kelas sama dengan dataset YOLO (class id 0..3)
CLASS_NAMES = ['red', 'green', 'orange', 'black']

# warna kotak di gambar debug (B, G, R)
BOX_COLORS = {
    'red': (40, 40, 230),
    'green': (60, 200, 60),
    'orange': (0, 140, 255),
    'black': (200, 200, 200),
}


def make_detection(header, class_name, score, cx, cy, w, h):
    """Satu deteksi: kelas, skor, dan kotak (pusat cx, cy dan ukuran w, h dalam pixel)."""
    det = Detection2D()
    det.header = header
    det.bbox.center.position.x = float(cx)
    det.bbox.center.position.y = float(cy)
    det.bbox.size_x = float(w)
    det.bbox.size_y = float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_name
    hyp.hypothesis.score = float(score)
    det.results.append(hyp)
    return det


def spin_until_ctrl_c(node):
    """Jalankan node sampai Ctrl+C. Ctrl+C hanya menyalakan penanda, lalu loop berhenti
    sendiri, jadi node tidak terpotong di tengah menerima atau mengirim pesan."""
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
