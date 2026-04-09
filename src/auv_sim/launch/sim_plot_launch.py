from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='auv_navigation',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),

        Node(
            package='auv_control',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),

        Node(
            package='auv_control',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),

        Node(
            package='auv_sim',
            executable='sim_vehicle_node',
            name='sim_vehicle_node',
            output='screen'
        ),

        Node(
            package='auv_sim',
            executable='plot_trajectory',
            name='plot_trajectory',
            output='screen'
        ),
    ])