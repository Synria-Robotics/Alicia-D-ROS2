"""Custom MoveIt config builder with robot version and gripper type support."""
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def get_versioned_moveit_config(
    robot_version='v5_6',
    gripper_type='50mm',
    port='',
    use_fake_hardware=False,
):
    """
    Build MoveIt configuration for specified robot version and gripper type.
    
    Args:
        robot_version: Robot version (v5_5, v5_6)
        gripper_type: Gripper type (50mm, 100mm, or "auto" for auto-detection)
        port: Serial port for hardware interface (empty string for auto-detection)
        baud_rate: Baud rate for serial communication (not used, fixed to 1000000)
        firmware_version: Firmware version (not used anymore, kept for compatibility)
    
    Returns:
        MoveItConfigs object
    """
    pkg_name = 'alicia_d_moveit'
    pkg_share = get_package_share_directory(pkg_name)
    
    # For URDF/SRDF, use "50mm" as default if "auto" (URDF needs a specific type)
    # The hardware interface will auto-detect the actual gripper type on connection
    gripper_type_for_urdf = "50mm" if gripper_type == "auto" else gripper_type
    
    # Build paths for versioned xacro (includes ros2_control)
    xacro_filename = (
        f'Alicia_D_{robot_version}_gripper_{gripper_type_for_urdf}_demo.urdf.xacro'
        if use_fake_hardware
        else f'Alicia_D_{robot_version}_gripper_{gripper_type_for_urdf}.urdf.xacro'
    )
    xacro_path = os.path.join(pkg_share, 'config', xacro_filename)
    
    srdf_path = os.path.join(
        pkg_share,
        'config',
        f'Alicia_D_{robot_version}_gripper_{gripper_type_for_urdf}.srdf'
    )
    
    # Xacro arguments for hardware interface configuration
    # Pass the original gripper_type (may be "auto") to hardware interface for auto-detection
    xacro_args = {
        'hw_port': port if port else '',  # Empty string for auto-detection
        'hw_gripper_type': gripper_type,  # Can be "auto", "50mm", or "100mm"
        'hw_default_speed_rad_s': '0.349',
    }
    
    # Build MoveIt config with versioned xacro
    moveit_config = (
        MoveItConfigsBuilder(f"Alicia_D_{robot_version}_gripper_{gripper_type_for_urdf}", package_name=pkg_name)
        .robot_description(file_path=xacro_path, mappings=xacro_args)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=os.path.join(pkg_share, "config/kinematics.yaml"))
        .joint_limits(file_path=os.path.join(pkg_share, "config/joint_limits.yaml"))
        .trajectory_execution(file_path=os.path.join(pkg_share, "config/moveit_controllers.yaml"))
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    return moveit_config
