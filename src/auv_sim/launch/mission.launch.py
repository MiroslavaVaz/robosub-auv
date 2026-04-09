#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # A* global planner
        Node(
            package='auv_sim',
            executable='a_star_planner',
            name='a_star_planner',
            output='screen'
        ),

        # Navigator: converts path -> velocity / heading commands
        Node(
            package='auv_sim',
            executable='navigator',
            name='navigator',
            output='screen'
        ),

        # Low-level controller: PID + thrusters
        Node(
            package='auv_sim',
            executable='auv_controller',
            name='auv_controller',
            output='screen'
        ),

        # Path follower: publishes TF/odom to move base_link along the path
        Node(
            package='auv_sim',
            executable='path_follower',
            name='path_follower',
            output='screen'
        ),

        # Environment markers: RoboSub gate, pipes, path markers
        Node(
            package='auv_sim',
            executable='env_markers',
            name='env_markers',
            output='screen'
        ),

        # RViz2  visualization 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
