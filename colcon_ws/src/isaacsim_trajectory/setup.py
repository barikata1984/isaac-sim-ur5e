from setuptools import setup, find_packages
import os

package_name = 'isaacsim_trajectory'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={package_name: 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='Trajectory generation package for UR5e in Isaac Sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator = isaacsim_trajectory.generator_cli:main',
        ],
    },
)
