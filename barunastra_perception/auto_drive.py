"""Menjalankan WAM-V otomatis menyusuri lintasan (alat bantu demo, BUKAN materi perception).

Posisi dari GPS dan arah dari IMU WAM-V. Titik tujuan ditulis relatif terhadap posisi
dan arah kapal saat node dijalankan (x maju, y kiri), jadi jalankan node ini saat
kapal masih di posisi awal. Mengirim /cmd_vel, jadi twist_to_thrust harus jalan.

    ros2 run barunastra_perception auto_drive
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, NavSatFix

from barunastra_perception.common import spin_until_ctrl_c

# tengah gate dan jalur di samping obstacle, koordinat lintasan (meter)
WAYPOINTS = [(15.0, 0.0), (22.0, -3.0), (30.0, 1.0), (37.0, -2.0),
             (45.0, 2.0), (52.0, 1.2), (60.0, 0.0), (70.0, 0.0)]
EARTH_R = 6371000.0


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')
        self.declare_parameter('speed', 0.6)        # nilai linear.x untuk twist_to_thrust
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.on_imu, qos_profile_sensor_data)
        self.origin = None      # (lat, lon, yaw) saat mulai
        self.yaw = None
        self.i = 0

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_gps(self, msg):
        if self.yaw is None:
            return
        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude, self.yaw)
            self.get_logger().info('titik awal tersimpan, mulai jalan')
        lat0, lon0, yaw0 = self.origin
        # GPS -> meter timur (e) dan utara (n) dari titik awal (cukup akurat untuk jarak pendek)
        e = math.radians(msg.longitude - lon0) * EARTH_R * math.cos(math.radians(lat0))
        n = math.radians(msg.latitude - lat0) * EARTH_R
        # ke koordinat lintasan: x searah haluan awal, y ke kiri
        x = math.cos(yaw0) * e + math.sin(yaw0) * n
        y = -math.sin(yaw0) * e + math.cos(yaw0) * n
        heading = self.yaw - yaw0

        cmd = Twist()
        if self.i < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.i]
            if math.hypot(tx - x, ty - y) < 2.5:
                self.get_logger().info(f'titik {self.i + 1}/{len(WAYPOINTS)} tercapai')
                self.i += 1
            err = math.atan2(ty - y, tx - x) - heading
            err = math.atan2(math.sin(err), math.cos(err))
            cmd.angular.z = max(-1.0, min(1.0, 1.5 * err))
            cmd.linear.x = float(self.get_parameter('speed').value) * max(0.0, math.cos(err))
        self.pub.publish(cmd)


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutoDrive()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
