from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='your_email@example.com',
    description='AUV hardware sensor nodes for depth, IMU, hydrophone, and logging',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_node = auv_hardware.depth_node:main',
            'imu_node = auv_hardware.imu_node:main',
            'hydrophone_node = auv_hardware.hydrophone_node:main',
            'data_logger_node = auv_hardware.data_logger_node:main',
        ],
    },
)