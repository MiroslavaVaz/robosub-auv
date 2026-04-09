#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Installed share directory for this package
    pkg_share = get_package_share_directory('auv_sim')

    # Paths in the installed package
    world_file = os.path.join(pkg_share, 'worlds', 'underwater.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'auv.urdf.xacro')

    # Process the xacro file in Python to get a URDF XML string
    xacro_doc = xacro.process_file(xacro_file)
    robot_desc = xacro_doc.toxml()
    robot_description = {'robot_description': robot_desc}

    # Start Gazebo Harmonic with the underwater world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # Robot State Publisher (publishes TF and robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Spawn AUV into Gazebo using ros_gz_sim
    spawn_auv = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'auv',
            '-topic', 'robot_description'
            '-x', '-7.5',     # spawn slightly in front of the start cube
            '-y', '0.0',
            '-z', '0.3',
            '-Y', '0.0'    #yaw orientation
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_auv,
    ])



