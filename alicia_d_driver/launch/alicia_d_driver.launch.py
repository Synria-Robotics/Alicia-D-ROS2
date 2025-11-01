from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the C++ Alicia D driver node."""
    
    alicia_driver_node = Node(
        package='alicia_d_driver',
        executable='alicia_d_driver_node',
        name='alicia_d_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'port': '/dev/ttyCH341USB0'},
            {'baud_rate': 1000000},
            {'debug_mode': True},
            {'servo_count': 9},
            {'rate_limit_sec': 0.01},
            {'command_rate_hz': 200.0},
            {'firmware_version': 'auto'},
            {'gripper_type': '50mm'},
            {'default_speed_rad_s': 0.349}
        ]
    )

    return LaunchDescription([
        alicia_driver_node
    ])