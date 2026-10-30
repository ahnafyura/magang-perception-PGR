from glob import glob

from setuptools import find_packages, setup

package_name = 'barunastra_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Barunastra ITS',
    maintainer_email='ahnafyura@users.noreply.github.com',
    description='Simulasi dan latihan perception Magang PGR Barunastra ITS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hsv_tuner = barunastra_perception.hsv_tuner:main',
            'hsv_detector = barunastra_perception.hsv_detector:main',
            'yolo_detector = barunastra_perception.yolo_detector:main',
            'dataset_capture = barunastra_perception.dataset_capture:main',
            'cloud_obstacles = barunastra_perception.cloud_obstacles:main',
            'buoy_ranger = barunastra_perception.buoy_ranger:main',
            'twist_to_thrust = barunastra_perception.twist_to_thrust:main',
            'auto_drive = barunastra_perception.auto_drive:main',
        ],
    },
)
