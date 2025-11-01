"""Launch file for controlling real Alicia-D robot with MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
import sys
import os
sys.path.append(os.path.dirname(__file__))
from moveit_config_builder import get_versioned_moveit_config


def launch_setup(context, *args, **kwargs):
    """Setup real robot launch with versioned config."""
    # Get launch configuration values
    robot_version = LaunchConfiguration('robot_version').perform(context)
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    port = LaunchConfiguration('port').perform(context)
    baud_rate = LaunchConfiguration('baud_rate').perform(context)
    firmware_version = LaunchConfiguration('firmware_version').perform(context)
    
    print(f'\033[1;32m[INFO] Launching real robot with version: {robot_version}, gripper: {gripper_type}\033[0m')
    print(f'\033[1;32m[INFO] Serial port: {port}, baud rate: {baud_rate}\033[0m')
    print(f'\033[1;32m[INFO] Firmware version: {firmware_version}\033[0m')
    
    # Get versioned MoveIt config with hardware parameters
    moveit_config = get_versioned_moveit_config(robot_version, gripper_type, port, baud_rate, firmware_version)
    
    # Update robot description with hardware interface parameters
    robot_description = moveit_config.robot_description
    
    # Controller manager node
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
    
    # Generate move_group launch (with fake_execution=false for real robot)
    move_group_params = {
        "allow_trajectory_execution": True,
        "fake_execution": False,
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
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("alicia_d_moveit"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
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
    
    nodes_to_start = [
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        delay_move_group,
        rviz_node,
    ]
    
    return nodes_to_start


def generate_launch_description():
    """Generate launch description for real robot control."""
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
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port for robot connection (e.g., /dev/ttyUSB0 or /dev/ttyCH341USB0)'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'firmware_version',
            default_value='auto',
            description='Firmware version (e.g., "5.0.0", "6.0.0", or "auto" for auto-detection)'
        ),
        OpaqueFunction(function=launch_setup)
    ])

