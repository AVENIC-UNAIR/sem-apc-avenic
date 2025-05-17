#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Parameter untuk controller
    controller_params = {
        'target_speed': 5.0,
        'waypoint_tolerance': 3.0,
        'kp': 0.2,
        'ki': 0.0,
        'kd': 0.15,
    }
    
    # Parameter untuk filter
    filter_params = {
        'deadband_threshold': 0.15,
        'lowpass_alpha': 0.1,
        'max_steering_change': 0.05,
    }
    
    # Parameter untuk waypoint
    waypoint_params = {
        'waypoint_spacing': 15.0,
        'lookahead_distance': 20.0,
    }
    
    # Node untuk waypoint manager
    waypoint_manager = Node(
        package='shell_simulation',
        executable='waypoint_manager.py',
        name='waypoint_manager',
        parameters=[waypoint_params],
        output='screen'
    )
    
    # Node untuk sensor fusion
    sensor_fusion = Node(
        package='shell_simulation',
        executable='sensor_fusion.py',
        name='sensor_fusion',
        parameters=[{'odom_weight': 0.95}],
        output='screen'
    )
    
    # Node untuk main controller
    main_controller = Node(
        package='shell_simulation',
        executable='straight_line_controller.py',
        name='straight_line_controller',
        parameters=[controller_params],
        output='screen'
    )
    
    # Node untuk stability filter
    stability_filter = Node(
        package='shell_simulation',
        executable='stability_filter.py',
        name='stability_filter',
        parameters=[filter_params],
        output='screen'
    )
    
    return LaunchDescription([
        waypoint_manager,
        sensor_fusion,
        main_controller,
        stability_filter
    ]) 