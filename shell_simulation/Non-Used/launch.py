#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path navigator node - calculates optimal path
    path_navigator = Node(
        package='shell_simulation',
        executable='path_navigator.py',
        name='path_navigator',
        output='screen'
    )
    
    # Vehicle controller node - follows the path
    vehicle_controller = Node(
        package='shell_simulation',
        executable='vehicle_controller.py',
        name='vehicle_controller',
        output='screen'
    )
    
    return LaunchDescription([
        path_navigator,
        vehicle_controller
    ])