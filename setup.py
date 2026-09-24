from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'px4_nav2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a0929370607@gmail.com',
    description='Nav2 to PX4 Offboard bridge for indoor (no-GPS) minimal SITL navigation testing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = px4_nav2_bridge.cmd_vel_bridge:main',
            'px4_odom_tf = px4_nav2_bridge.px4_odom_tf:main',
        ],
    },
)
