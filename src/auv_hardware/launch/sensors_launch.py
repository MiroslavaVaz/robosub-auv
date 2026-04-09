from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='auv_hardware',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        Node(
            package='auv_hardware',
            executable='depth_node',
            name='depth_node',
            output='screen'
        ),
        Node(
            package='auv_hardware',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='auv_hardware',
            executable='hydrophone_node',
            name='hydrophone_node',
            output='screen'
        ),
        Node(
            package='auv_hardware',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen'
        ),
        
    ])