"""Mencari obstacle dari point cloud LiDAR 3D WAM-V.

Langkah:
  1. buang titik yang tidak valid, terlalu dekat (badan kapal), terlalu jauh,
     di bawah permukaan air, atau terlalu tinggi (pohon); hanya bagian depan kapal
  2. jarak terdekat di sektor kiri, depan, kanan
  3. kelompokkan titik yang berdekatan menjadi objek (clustering)

Subscribe : /wamv/sensors/lidars/lidar_wamv_sensor/points  (sensor_msgs/PointCloud2)
Publish   : /perception/lidar/obstacles     (sensor_msgs/PointCloud2, titik yang lolos filter)
            /perception/lidar/sectors       (std_msgs/Float32MultiArray: [kiri, depan, kanan] meter)
            /perception/lidar/front_blocked (std_msgs/Bool)
            /perception/lidar/clusters      (visualization_msgs/MarkerArray)

Frame LiDAR: x ke depan, y ke kiri, z ke atas. LiDAR WAM-V dipasang 1,8 m di atas
base_link, jadi permukaan air ada di sekitar z = -1,7 m.

Jalankan:
    ros2 run barunastra_perception cloud_obstacles
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from barunastra_perception.common import CLOUD_TOPIC, spin_until_ctrl_c


def filter_points(pts, z_min, z_max, r_min, r_max):
    """pts: array N x 3 (x, y, z). Kembalikan hanya titik yang valid dan berada di
    rentang tinggi [z_min, z_max] serta jarak datar [r_min, r_max]."""
    r = np.hypot(pts[:, 0], pts[:, 1])
    # TODO 5: selain titik valid (isfinite), simpan hanya titik dengan
    #         z_min <= z <= z_max dan r_min <= r <= r_max. Gabungkan syarat dengan &.
    keep = np.isfinite(pts).all(axis=1)
    return pts[keep]


def sector_min(pts, start_deg, end_deg):
    """Jarak datar terdekat dari titik yang sudutnya di [start_deg, end_deg].
    Sudut 0 = depan, positif = kiri. Tidak ada titik = inf."""
    # TODO 6: hitung sudut tiap titik dalam derajat (0 = depan, positif = kiri).
    #         Petunjuk: np.arctan2(y, x) lalu np.degrees.
    angle = np.zeros(len(pts))
    r = np.hypot(pts[:, 0], pts[:, 1])
    inside = (angle >= start_deg) & (angle <= end_deg)
    return float(r[inside].min()) if inside.any() else math.inf


def cluster(pts, gap):
    """Euclidean clustering sederhana di bidang datar (x, y): titik yang jaraknya
    < gap dari salah satu anggota masuk kelompok yang sama. Kembalikan list indeks."""
    xy = pts[:, :2]
    unvisited = set(range(len(xy)))
    groups = []
    while unvisited:
        seed = unvisited.pop()
        group, queue = [seed], [seed]
        while queue:
            i = queue.pop()
            near = [j for j in unvisited if np.hypot(*(xy[j] - xy[i])) < gap]
            for j in near:
                unvisited.remove(j)
            group += near
            queue += near
        groups.append(group)
    return groups


class CloudObstacles(Node):
    def __init__(self):
        super().__init__('cloud_obstacles')
        self.declare_parameter('cloud_topic', CLOUD_TOPIC)
        self.declare_parameter('z_min', -1.6)            # sedikit di atas permukaan air
        self.declare_parameter('z_max', 1.0)             # di bawah pucuk pohon
        self.declare_parameter('r_min', 4.0)             # badan WAM-V sendiri
        self.declare_parameter('r_max', 60.0)
        self.declare_parameter('fov_half', 100.0)        # derajat, abaikan daratan di belakang kapal
        self.declare_parameter('front_half_angle', 15.0)
        self.declare_parameter('safe_distance', 10.0)
        self.declare_parameter('gap', 1.0)               # jarak maksimal antar titik satu objek
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(PointCloud2, '/perception/lidar/obstacles', 10)
        self.pub_sec = self.create_publisher(Float32MultiArray, '/perception/lidar/sectors', 10)
        self.pub_block = self.create_publisher(Bool, '/perception/lidar/front_blocked', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/perception/lidar/clusters', 10)
        self.count = 0

    def p(self, name):
        return float(self.get_parameter(name).value)

    def on_cloud(self, msg):
        raw = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        pts = filter_points(raw, self.p('z_min'), self.p('z_max'), self.p('r_min'), self.p('r_max'))
        pts = pts[np.abs(np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))) <= self.p('fov_half')]
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(msg.header, pts.tolist()))

        f = self.p('front_half_angle')
        left, front, right = sector_min(pts, f, 90), sector_min(pts, -f, f), sector_min(pts, -90, -f)
        self.pub_sec.publish(Float32MultiArray(data=[left, front, right]))
        blocked = front < self.p('safe_distance')
        self.pub_block.publish(Bool(data=blocked))

        groups = cluster(pts, self.p('gap')) if len(pts) < 3000 else []
        self.pub_mk.publish(self.markers(msg.header, pts, groups))

        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f'{len(pts)} titik, {len(groups)} objek | kiri {left:5.1f} m | depan {front:5.1f} m'
                f' | kanan {right:5.1f} m' + ('  << HALANGAN DI DEPAN' if blocked else ''))

    def markers(self, header, pts, groups):
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        for n, g in enumerate(groups):
            c = pts[g].mean(axis=0)
            dist = math.hypot(c[0], c[1])
            ball = Marker()
            ball.header = header
            ball.ns, ball.id, ball.type = 'objek', n, Marker.SPHERE
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = map(float, c)
            ball.pose.orientation.w = 1.0
            ball.scale.x = ball.scale.y = ball.scale.z = 0.8
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.45, 0.0, 0.8
            text = Marker()
            text.header = header
            text.ns, text.id, text.type = 'jarak', n, Marker.TEXT_VIEW_FACING
            # digeser ke kanan bola (y negatif), label fusion digeser ke kiri, supaya tidak bertumpuk
            text.pose.position.x, text.pose.position.y = float(c[0]), float(c[1]) - 1.8
            text.pose.position.z = float(c[2])
            text.pose.orientation.w = 1.0
            text.scale.z = 0.8
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'{dist:.1f} m ({len(g)} titik)'
            out.markers += [ball, text]
        return out


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CloudObstacles()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
