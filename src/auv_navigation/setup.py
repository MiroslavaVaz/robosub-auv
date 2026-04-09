from setuptools import find_packages, setup

package_name = 'auv_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miroslava',
    maintainer_email='your_email@example.com',
    description='AUV navigation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_node = auv_navigation.state_node:main',
            'path_planner_node = auv_navigation.path_planner_node:main',
            'navigator = auv_navigation.navigator:main',
        ],
    },
)