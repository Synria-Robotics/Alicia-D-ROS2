from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """Setup launch with parameters."""
    # Get launch configuration values
    port = LaunchConfiguration('port').perform(context)
    baud_rate = int(LaunchConfiguration('baud_rate').perform(context))
    debug_mode_str = LaunchConfiguration('debug_mode').perform(context)
    debug_mode = debug_mode_str.lower() in ('true', '1', 'yes', 'on')
    gripper_type = LaunchConfiguration('gripper_type').perform(context)
    firmware_version = LaunchConfiguration('firmware_version').perform(context)
    
    alicia_driver_node = Node(
        package='alicia_d_driver',
        executable='alicia_d_driver_node',
        name='alicia_d_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': port,
            'baud_rate': baud_rate,
            'debug_mode': debug_mode,
            'servo_count': 9,
            'rate_limit_sec': 0.01,
            'command_rate_hz': 200.0,
            'firmware_version': firmware_version,
            'gripper_type': gripper_type,
            'default_speed_rad_s': 0.349
        }]
    )

    return [alicia_driver_node]

def generate_launch_description():
    """Launch the Alicia D driver node."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for robot connection (e.g., /dev/ttyUSB0 or /dev/ttyCH341USB0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Baud rate for serial communication'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode for verbose output'
    )
    
    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='50mm',
        description='Gripper type: 50mm or 100mm'
    )
    
    firmware_version_arg = DeclareLaunchArgument(
        'firmware_version',
        default_value='auto',
        description='Firmware version (e.g., "5.0.0", "6.0.0", or "auto" for auto-detection)'
    )

    return LaunchDescription([
        port_arg,
        baud_rate_arg,
        debug_mode_arg,
        gripper_type_arg,
        firmware_version_arg,
        OpaqueFunction(function=launch_setup)
    ])