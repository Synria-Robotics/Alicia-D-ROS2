"""Custom MoveIt config builder with robot version and gripper type support."""
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def get_versioned_moveit_config(robot_version='v5_6', gripper_type='50mm'):
    """
    Build MoveIt configuration for specified robot version and gripper type.
    
    Args:
        robot_version: Robot version (v5_5, v5_6)
        gripper_type: Gripper type (50mm, 100mm)
    
    Returns:
        MoveItConfigs object
    """
    pkg_name = 'alicia_d_moveit'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Build paths for versioned xacro (includes ros2_control)
    xacro_path = os.path.join(
        pkg_share,
        'config',
        f'Alicia_D_{robot_version}_gripper_{gripper_type}.urdf.xacro'
    )
    
    srdf_path = os.path.join(
        pkg_share,
        'config',
        f'Alicia_D_{robot_version}_gripper_{gripper_type}.srdf'
    )
    
    # Build MoveIt config with versioned xacro
    moveit_config = (
        MoveItConfigsBuilder(f"Alicia_D_{robot_version}_gripper_{gripper_type}", package_name=pkg_name)
        .robot_description(file_path=xacro_path)
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
