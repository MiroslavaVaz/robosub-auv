from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_vision'

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
    description='AUV vision package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_cnn_live = auv_vision.camera_cnn_live:main',
            'predict_cnn = auv_vision.predict_cnn:main',
            'train_cnn = auv_vision.train_cnn:main',
            'test_camera = auv_vision.test_camera:main',
            'image_augmentation = auv_vision.image_augmentation:main',
            'image_videoextraction = auv_vision.image_videoextraction:main',
            'vision_mission_node = auv_vision.vision_mission_node:main',
        ],
    },
)