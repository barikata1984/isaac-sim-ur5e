from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'isaacsim_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='Isaac Sim GUI launcher with modular isaacsim_core package',
    license='MIT',
    tests_require=['pytest'],
    scripts=['scripts/isaac_sim_gui.sh'],
    entry_points={
        'console_scripts': [
            'isaac_sim_node = isaacsim_core.main:main',
        ],
    },
)
