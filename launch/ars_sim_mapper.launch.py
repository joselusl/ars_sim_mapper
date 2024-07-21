#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'ars_sim_mapper_node_name', default_value='ars_sim_mapper_node',
            description='Name of the sim mapper node'
        ),
        DeclareLaunchArgument(
            'screen', default_value='screen',
            description='Output setting for the nodes'
        ),
        
        # Launch the node
        Node(
            package='ars_sim_mapper',
            executable='ars_sim_mapper_ros_node',
            name=LaunchConfiguration('ars_sim_mapper_node_name'),
            output=LaunchConfiguration('screen')
        ),
        
    ])
