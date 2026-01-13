"""Demo launch file for Alicia-D MoveIt with version and gripper type selection."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import sys
import os
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup demo launch with versioned config."""
    # Get launch configuration values
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    
    
    # Get versioned MoveIt config using fake hardware for demo
    moveit_config = get_versioned_moveit_config(
        gripper_type,
        use_fake_hardware=True,
    )
    
    # Build path to demo.rviz using source directory (not install directory)
    package_share = get_package_share_directory('alicia_d_moveit')
    # Try source directory first
    source_rviz = os.path.join(os.path.dirname(package_share), '..', '..', '..', 'src', 'alicia_d_moveit', 'config', 'demo.rviz')
    source_rviz = os.path.abspath(source_rviz)
    
    # Fall back to install directory if source not found
    if os.path.exists(source_rviz):
        rviz_config_file = source_rviz
    else:
        rviz_config_file = os.path.join(package_share, 'config', 'demo.rviz')
    
    # Verify the file exists
    if not os.path.exists(rviz_config_file):
        print(f'\033[1;31m[ERROR] RViz config file not found: {rviz_config_file}\033[0m')
    else:
        print(f'\033[1;32m[INFO] Using RViz config: {rviz_config_file}\033[0m')
    
    # Get robot description
    robot_description = moveit_config.robot_description
    
    # Controller manager node (required for controllers to work)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("alicia_d_moveit"),
                "config",
                "ros2_controllers.yaml"
            ]),
        ],
        output="screen",
    )
    
    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Spawner for Alicia_controller (arm)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Alicia_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Spawner for Gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Move group node (with fake_execution=true for demo)
    move_group_params = {
        "allow_trajectory_execution": True,
        "fake_execution": True,  # Demo mode uses fake execution
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
    }
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_params,
        ],
    )
    
    # Custom RViz node using demo.rviz
    custom_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # Delay arm and gripper controller spawners until joint_state_broadcaster is loaded
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    
    # Delay move_group until controllers are loaded
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_node],
        )
    )
    
    # Build list of entities to return
    entities = [
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        delay_move_group,
        custom_rviz_node,
    ]
    
    return entities


def generate_launch_description():
    """Generate launch description with robot  gripper type arguments."""
    return LaunchDescription([

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
