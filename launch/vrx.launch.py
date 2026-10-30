"""Menjalankan VRX (WAM-V di danau Sydney Regatta + lintasan buoy), twist_to_thrust, dan RViz.

Contoh:
    ros2 launch barunastra_perception vrx.launch.py
    ros2 launch barunastra_perception vrx.launch.py headless:=true rviz:=false   # laptop berat
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('barunastra_perception')
    world = os.path.join(pkg, 'worlds', 'perception_regatta.sdf')

    vrx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vrx_gz'), 'launch', 'competition.launch.py')),
        launch_arguments={'world': world, 'headless': LaunchConfiguration('headless')}.items())

    thrust = Node(package='barunastra_perception', executable='twist_to_thrust',
                  parameters=[{'use_sim_time': True}], output='screen')

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', os.path.join(pkg, 'config', 'perception.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False',
                              description='True = tanpa jendela Gazebo (lebih ringan)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        vrx, thrust, rviz,
    ])
