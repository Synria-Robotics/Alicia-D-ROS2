#!/usr/bin/env python3
"""
Cube Sorting Launch File

This launch file starts the cube sorting node which controls the robot arm
to sort cubes by color into designated drop zones.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description"""
    
    # Find config file
    try:
        pkg_share = get_package_share_directory('alicia_d_cube_sort')
        config_file = os.path.join(pkg_share, 'config', 'cube_sorting.yaml')
    except Exception:
        config_file = ''
    
    # Launch arguments
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start sorting workflow'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera optical frame'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )
    
    # Cube sorting node
    cube_sorter_node = Node(
        package='alicia_d_cube_sort',
        executable='cube_sorting.py',
        name='cube_sorter',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'auto_start': LaunchConfiguration('auto_start'),
            'base_frame': LaunchConfiguration('base_frame'),
            'camera_frame': LaunchConfiguration('camera_frame')
        }]
    )
    
    return LaunchDescription([
        auto_start_arg,
        base_frame_arg,
        camera_frame_arg,
        config_file_arg,
        cube_sorter_node
    ])
