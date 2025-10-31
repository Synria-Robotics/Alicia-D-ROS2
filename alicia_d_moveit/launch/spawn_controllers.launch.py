"""Spawn controllers launch file for Alicia-D with version and gripper type selection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_spawn_controllers_launch
import sys
import os
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup spawn controllers launch with versioned config."""
    # Get launch configuration values
    robot_version = LaunchConfiguration('robot_version').perform(context)
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    
    # Get versioned MoveIt config
    moveit_config = get_versioned_moveit_config(robot_version, gripper_type)
    
    # Generate standard spawn controllers launch
    controller_nodes = generate_spawn_controllers_launch(moveit_config)
    
    return controller_nodes.entities


def generate_launch_description():
    """Generate launch description with robot version and gripper type arguments."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_version',
            default_value='v5_6',
            description='Robot version: v5_5 or v5_6'
        ),
        DeclareLaunchArgument(
            'gripper_type',
            default_value='50mm',
            description='Gripper type: 50mm or 100mm'
        ),
        OpaqueFunction(function=launch_setup)
    ])
