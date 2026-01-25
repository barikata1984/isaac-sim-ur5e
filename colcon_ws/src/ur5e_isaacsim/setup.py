from setuptools import setup
import os
from glob import glob

package_name = 'ur5e_isaacsim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='Isaac Sim GUI launcher with UR5e robot spawning',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/spawn_ur5e_wrapper.sh'],
    entry_points={
        'console_scripts': [
            'spawn_ur5e_node = ur5e_isaacsim.spawn_ur5e_node:main',
        ],
    },
)
