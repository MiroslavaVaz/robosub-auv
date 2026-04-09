from setuptools import setup
import os
from glob import glob

package_name = 'auv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miroslava',
    maintainer_email='miroslava@todo.todo',
    description='AUV Simulation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environment_markers = auv_sim.environment_markers:main',
            'sim_vehicle_node = auv_sim.sim_vehicle_node:main',
            'plot_trajectory = auv_sim.plot_trajectory:main',
        ],
    },
)