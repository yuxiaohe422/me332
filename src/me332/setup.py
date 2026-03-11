from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'me332'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'world'), glob('world/*.wbt')),
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),
        (os.path.join('share', package_name, 'world'), glob('world/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yxh',
    maintainer_email='myoevans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_avoider = me332.obstacle_avoider:main',
            'hand_gesture_teleop = me332.hand_gesture:main',
            'voice_teleop = me332.voice_teleop:main',
            'robot_description_publisher = me332.robot_description_publisher:main',
            'odom_to_tf_publisher = me332.odom_to_tf_publisher:main',
            'arm_control = me332.arm_control:main',
            'arm_teleop = me332.arm_teleop:main',
        ],
    },
)
