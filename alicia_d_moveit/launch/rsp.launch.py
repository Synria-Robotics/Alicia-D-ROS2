"""Robot state publisher launch file for Alicia-D with version and gripper type selection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_rsp_launch
import sys
import os
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup robot state publisher launch with versioned config."""
    # Get launch configuration values
    robot_version = LaunchConfiguration('robot_version').perform(context)
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    
    print(f'\033[1;32m[INFO] Starting robot_state_publisher with robot version: {robot_version}, gripper type: {gripper_type}\033[0m')
    
    # Get versioned MoveIt config
    moveit_config = get_versioned_moveit_config(robot_version, gripper_type)
    
    # Generate standard RSP launch
    rsp_nodes = generate_rsp_launch(moveit_config)
    
    return rsp_nodes.entities


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
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='15.0',
            description='Publishing frequency for robot_state_publisher'
        ),
        OpaqueFunction(function=launch_setup)
    ])
