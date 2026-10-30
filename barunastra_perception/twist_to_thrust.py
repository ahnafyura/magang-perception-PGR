"""Mengubah /cmd_vel (geometry_msgs/Twist) menjadi gaya thruster kiri dan kanan WAM-V.

Supaya kapal VRX bisa digerakkan dengan teleop_twist_keyboard:
    ros2 run barunastra_perception twist_to_thrust
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

thrust kiri  = k_linear * v - k_angular * w
thrust kanan = k_linear * v + k_angular * w      (Newton, dibatasi +-max_thrust)
Kalau /cmd_vel berhenti datang lebih dari 1 detik, thruster dinolkan.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from barunastra_perception.common import spin_until_ctrl_c


class TwistToThrust(Node):
    def __init__(self):
        super().__init__('twist_to_thrust')
        self.declare_parameter('k_linear', 800.0)
        self.declare_parameter('k_angular', 500.0)
        self.declare_parameter('max_thrust', 1000.0)
        self.left = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.cmd = Twist()
        self.last = self.get_clock().now()
        self.create_timer(0.05, self.tick)      # thruster VRX perlu perintah terus-menerus

    def on_cmd(self, msg):
        self.cmd = msg
        self.last = self.get_clock().now()

    def tick(self):
        v, w = self.cmd.linear.x, self.cmd.angular.z
        if (self.get_clock().now() - self.last).nanoseconds > 1e9:
            v = w = 0.0
        kl = float(self.get_parameter('k_linear').value)
        ka = float(self.get_parameter('k_angular').value)
        m = float(self.get_parameter('max_thrust').value)
        self.left.publish(Float64(data=max(-m, min(m, kl * v - ka * w))))
        self.right.publish(Float64(data=max(-m, min(m, kl * v + ka * w))))


def main():
    # Ctrl+C ditangani sendiri oleh spin_until_ctrl_c (lihat common.py)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TwistToThrust()
    try:
        spin_until_ctrl_c(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left.publish(Float64(data=0.0))     # matikan thruster saat node ditutup
        node.right.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
