from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_tutorial_pkgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='ROS 2 tutorial package for UR5e in Isaac Sim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_control = scripts.ros2_control:main',
            'simple_control = scripts.simple_control:main',
            'target_publisher = scripts.target_publisher:main',
        ],
    },
)
