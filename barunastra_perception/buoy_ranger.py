"""Fusion kamera + LiDAR: kamera tahu WARNA dan letak buoy di gambar, LiDAR tahu JARAKNYA.

Langkah per frame:
  1. titik LiDAR (sudah difilter) dipindah ke frame kamera memakai TF
  2. tiap titik diproyeksikan ke pixel (u, v) dengan model pinhole (fx, fy, cx, cy dari CameraInfo)
  3. untuk tiap deteksi, ambil titik yang jatuh di dalam bounding box
  4. titik-titik itu dikelompokkan menurut kedalaman; dipilih kelompok yang paling
     cocok dengan perkiraan jarak dari tinggi kotak (buoy di depan buoy lain bisa
     ikut masuk kotak). Posisi buoy = rata-rata kelompok itu.

Subscribe : <detections_topic>  (vision_msgs/Detection2DArray, default dari YOLO)
            camera_info kamera depan kiri, point cloud LiDAR, TF
Publish   : /perception/buoys          (vision_msgs/Detection2DArray, pose = posisi di base_link)
            /perception/buoys/markers  (visualization_msgs/MarkerArray untuk RViz)

Jalankan:
    ros2 run barunastra_perception buoy_ranger
    ros2 run barunastra_perception buoy_ranger --ros-args -p detections_topic:=/perception/hsv/detections
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from barunastra_perception.cloud_obstacles import filter_points
from barunastra_perception.common import (BASE_FRAME, CAMERA_INFO_TOPIC, CLOUD_TOPIC,
                                          spin_until_ctrl_c)

# tinggi buoy yang terlihat di atas air (meter), dipakai untuk perkiraan jarak
OBJECT_HEIGHT = {'red': 0.9, 'green': 0.9, 'orange': 0.4, 'black': 0.4}

MARKER_RGB = {
    'red': (0.9, 0.1, 0.1), 'green': (0.1, 0.8, 0.2),
    'orange': (1.0, 0.5, 0.0), 'black': (0.15, 0.15, 0.15),
}


def transform_matrix(tf):
    """geometry_msgs/TransformStamped -> (R 3x3, t 3), sehingga p_target = R @ p_source + t."""
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    t = tf.transform.translation
    return R, np.array([t.x, t.y, t.z])


def project(pts_cam, fx, fy, cx, cy):
    """Titik di frame kamera (x maju, y kiri, z atas) -> pixel (u, v).
    Titik di belakang atau terlalu dekat kamera (x <= 0.5 m) diberi nan."""
    x, y, z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
    # TODO 7: model pinhole. Titik lurus di depan (y = 0, z = 0) jatuh di (cx, cy).
    #         Titik di KIRI (y > 0) jatuh di kiri gambar (u < cx), titik di ATAS (z > 0)
    #         jatuh di atas gambar (v < cy). Bagi dengan x (jarak ke depan).
    u = np.full(len(x), cx, dtype=float)
    v = np.full(len(x), cy, dtype=float)
    behind = x <= 0.5
    u[behind] = np.nan
    v[behind] = np.nan
    return u, v


def in_box(u, v, cx, cy, w, h):
    """Mask titik yang pixelnya di dalam kotak (pusat cx, cy, ukuran w, h)."""
    # TODO 8: True jika u di dalam [cx - w/2, cx + w/2] DAN v di dalam [cy - h/2, cy + h/2].
    return np.zeros(len(u), dtype=bool)


def pick_group(depths, expected=None, gap=1.0):
    """Kelompokkan nilai kedalaman yang berdekatan (selisih <= gap), lalu kembalikan
    indeks kelompok terpilih: yang mediannya paling dekat dengan `expected`,
    atau kelompok terdekat ke kamera bila expected None."""
    order = np.argsort(depths)
    groups, cur = [], [order[0]]
    for a, b in zip(order[:-1], order[1:]):
        if depths[b] - depths[a] > gap:
            groups.append(cur)
            cur = []
        cur.append(b)
    groups.append(cur)
    if expected is None:
        return np.array(groups[0])
    return np.array(min(groups, key=lambda g: abs(float(np.median(depths[g])) - expected)))


class BuoyRanger(Node):
    def __init__(self):
        super().__init__('buoy_ranger')
        self.declare_parameter('detections_topic', '/perception/yolo/detections')
        self.declare_parameter('use_size_estimate', True)   # False = ambil kelompok terdekat saja
        self.declare_parameter('tolerance', 0.5)            # selisih maksimal LiDAR vs perkiraan (relatif)
        self.declare_parameter('z_min', -1.6)
        self.declare_parameter('z_max', 1.0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud = None
        self.K = None
        topic = self.get_parameter('detections_topic').value
        self.create_subscription(Detection2DArray, topic, self.on_detections, 10)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection2DArray, '/perception/buoys', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/perception/buoys/markers', 10)
        self.count = 0
        self.get_logger().info(f'buoy_ranger memakai deteksi dari {topic}')

    def on_info(self, msg):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_cloud(self, msg):
        self.cloud = msg

    def lookup(self, target, source):
        try:
            return transform_matrix(self.tf_buffer.lookup_transform(target, source, Time()))
        except Exception as e:     # TF belum diterima di detik-detik awal
            self.get_logger().warn(f'TF {source} -> {target} belum ada: {e}', throttle_duration_sec=5)
            return None

    def on_detections(self, msg):
        if self.cloud is None or self.K is None or not msg.detections:
            return
        cam_frame, lidar_frame = msg.header.frame_id, self.cloud.header.frame_id
        to_cam, to_base = self.lookup(cam_frame, lidar_frame), self.lookup(BASE_FRAME, lidar_frame)
        if to_cam is None or to_base is None:
            return
        raw = point_cloud2.read_points_numpy(self.cloud, field_names=('x', 'y', 'z'))
        pts = filter_points(raw, float(self.get_parameter('z_min').value),
                            float(self.get_parameter('z_max').value), 3.0, 120.0)
        if len(pts) == 0:
            return
        fx, fy, cx, cy = self.K
        pts_cam = pts @ to_cam[0].T + to_cam[1]
        u, v = project(pts_cam, fx, fy, cx, cy)

        out = Detection2DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = BASE_FRAME
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        laporan = []

        for det in msg.detections:
            b = det.bbox
            inside = np.where(in_box(u, v, b.center.position.x, b.center.position.y,
                                     b.size_x, b.size_y))[0]
            if len(inside) == 0:
                continue
            name = det.results[0].hypothesis.class_id
            depths = pts_cam[inside, 0]
            expected = None
            if self.get_parameter('use_size_estimate').value and name in OBJECT_HEIGHT and b.size_y > 0:
                expected = fy * OBJECT_HEIGHT[name] / b.size_y     # pinhole: h_px = fy * H / kedalaman
            group = inside[pick_group(depths, expected)]
            depth = float(np.median(pts_cam[group, 0]))
            tol = float(self.get_parameter('tolerance').value)
            if expected is not None and abs(depth - expected) > tol * expected:
                continue
            p_base = pts[group].mean(axis=0) @ to_base[0].T + to_base[1]
            dist = math.hypot(p_base[0], p_base[1])

            det.header = out.header
            det.results[0].pose.pose.position.x = float(p_base[0])
            det.results[0].pose.pose.position.y = float(p_base[1])
            det.results[0].pose.pose.position.z = float(p_base[2])
            out.detections.append(det)
            laporan.append(f'{name} {dist:.1f} m')
            markers.markers += self.make_markers(len(out.detections), out.header, name, p_base, dist)

        self.pub.publish(out)
        self.pub_mk.publish(markers)
        self.count += 1
        if self.count % 10 == 0 and laporan:
            self.get_logger().info(' | '.join(laporan))

    def make_markers(self, i, header, name, p, dist):
        r, g, b = MARKER_RGB.get(name, (1.0, 1.0, 1.0))
        cyl = Marker()
        cyl.header = header
        cyl.ns, cyl.id, cyl.type = 'buoy', i, Marker.CYLINDER
        cyl.pose.position.x, cyl.pose.position.y = float(p[0]), float(p[1])
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = cyl.scale.y = 0.7
        cyl.scale.z = 1.2
        cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = r, g, b, 0.9
        txt = Marker()
        txt.header = header
        txt.ns, txt.id, txt.type = 'label', i, Marker.TEXT_VIEW_FACING
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(p[0]), float(p[1]) + 1.8, 1.6
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.9
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = f'{name} {dist:.1f} m'
        return [cyl, txt]


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BuoyRanger()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
