from setuptools import setup, find_packages

package_name = 'dynamics'

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
    install_requires=['setuptools', 'numpy', 'scipy', 'pymlg'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='Newton-Euler inverse dynamics for UR5e using twist-wrench formulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamics_cli = dynamics.dynamics_cli:main',
            'dynamics_verify = dynamics.offline_verification:run_comprehensive_verification',
            'dynamics_verify_node = dynamics.verification_node:main',
            'isaac_verification = dynamics.isaac_verification_node:main',
        ],
    },
)
