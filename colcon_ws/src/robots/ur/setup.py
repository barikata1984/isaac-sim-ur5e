"""Setup script for UR robot package."""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='UR robot spawning package for Isaac Sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ライブラリパッケージのため、エントリーポイントなし
        ],
    },
)
