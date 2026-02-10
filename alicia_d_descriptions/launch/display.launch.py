import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    """Setup function to resolve launch configurations at runtime"""
    pkg_dir = get_package_share_directory('alicia_d_descriptions')
    
    # Get launch config values
    robot_version = LaunchConfiguration('robot_version').perform(context)
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    
    # Build URDF path
    urdf_file = os.path.join(
        pkg_dir, 'urdf', 
        f'Alicia_D_{robot_version}',
        f'Alicia_D_{robot_version}_gripper_{gripper_type}.urdf'
    )
    
    # Log which URDF is being loaded
    print(f'\033[1;32m[INFO] Loading URDF: {urdf_file}\033[0m')
    print(f'\033[1;32m[INFO] Robot version: {robot_version}, Gripper type: {gripper_type}\033[0m')
    
    # Read URDF content
    if not os.path.exists(urdf_file):
        print(f'\033[1;31m[ERROR] URDF file not found: {urdf_file}\033[0m')
        raise FileNotFoundError(f'URDF file not found: {urdf_file}')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # robot_state_publisher publishes TF from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher (GUI or regular)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
    
    # Static TF world -> base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )
    
    # RViz with URDF file path as argument for Description Source: File
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config],
        parameters=[{'robot_description': robot_description}]
    )
    
    return [
        static_tf_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('alicia_d_descriptions')
    
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
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher GUI'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_dir, 'launch', 'alicia.rviz'),
            description='Path to RViz config file'
        ),
        OpaqueFunction(function=launch_setup)
    ])
