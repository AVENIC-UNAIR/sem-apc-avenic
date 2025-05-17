#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

# Example ROS launch file
def generate_launch_description():
    # Efficient navigator node that handles both path planning and vehicle control
    efficient_navigator = Node(
        package='shell_simulation',
        executable='efficient_navigator.py',
        name='efficient_navigator',
        output='screen'
    )
    
    return LaunchDescription([  
        # Nodes
        efficient_navigator
    ])