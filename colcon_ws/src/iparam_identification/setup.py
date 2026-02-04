from setuptools import setup, find_packages

package_name = 'iparam_identification'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.sensor',
        f'{package_name}.estimation',
        f'{package_name}.trajectory',
    ],
    package_dir={
        package_name: 'src',
        f'{package_name}.sensor': 'src/sensor',
        f'{package_name}.estimation': 'src/estimation',
        f'{package_name}.trajectory': 'src/trajectory',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='franche1984@gmail.com',
    description='Inertial parameter identification using recursive total least-squares',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
