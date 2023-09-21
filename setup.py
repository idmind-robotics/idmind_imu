import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'idmind_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'imu_razor'), glob('models/imu_razor/*.*')),
        (os.path.join('share', package_name, 'models', 'imu_brick_v2'), glob('models/imu_brick_v2/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Neves',
    maintainer_email='cneves@idmind.pt',
    description='IDMinds package to handle IMUs on robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_brick_node = idmind_imu.imu_brick_node:main',
        ],
    },
)
