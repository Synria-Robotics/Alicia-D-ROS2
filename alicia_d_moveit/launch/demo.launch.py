"""Demo launch file for Alicia-D MoveIt with version and gripper type selection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
import sys
import os
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup demo launch with versioned config."""
    # Get launch configuration values
    robot_version = LaunchConfiguration('robot_version').perform(context)
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    
    print(f'\033[1;32m[INFO] Starting MoveIt demo with robot version: {robot_version}, gripper type: {gripper_type}\033[0m')
    
    # Get versioned MoveIt config
    moveit_config = get_versioned_moveit_config(robot_version, gripper_type)
    
    # Build path to demo.rviz using package_path from moveit_config
    rviz_config_file = os.path.join(moveit_config.package_path, 'config', 'demo.rviz')
    
    # Generate standard demo launch
    demo_nodes = generate_demo_launch(moveit_config)
    
    # Filter out the default RViz node and replace with ours using demo.rviz
    filtered_entities = []
    
    for entity in demo_nodes.entities:
        # Check if this is the RViz node
        is_rviz = False
        if hasattr(entity, '_Node__node_executable'):
            is_rviz = (entity._Node__node_executable == 'rviz2')
        elif hasattr(entity, '_Node__node_name'):
            is_rviz = ('rviz' in str(entity._Node__node_name).lower())
        
        if not is_rviz:
            filtered_entities.append(entity)
        else:
            # Replace with custom RViz node using demo.rviz
            custom_rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ]
            )
            filtered_entities.append(custom_rviz_node)
    
    return filtered_entities


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
            'db',
            default_value='false',
            description='Start database'
        ),
        OpaqueFunction(function=launch_setup)
    ])
