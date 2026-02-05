"""
Utility modules for 6D grasp pipeline (Intel RealSense D405).
"""

from .transform_utils import (
    load_hand_eye_calibration,
    transform_pose_to_base,
    align_gripper_convention,
    apply_gripper_offset,
    pose_to_matrix,
    matrix_to_pose,
    filter_grasp_poses
)

from .camera_utils import (
    CameraParameterManager,
    get_camera_intrinsics_from_topic,
    get_tf_transform,
    DEFAULT_IR_K,
    DEFAULT_RGB_K,
    DEFAULT_T_IR_TO_RGB,
    DEFAULT_IR_RESOLUTION,
    DEFAULT_RGB_RESOLUTION,
    DEFAULT_BASELINE
)

__all__ = [
    # Transform utilities
    'load_hand_eye_calibration',
    'transform_pose_to_base',
    'align_gripper_convention',
    'apply_gripper_offset',
    'pose_to_matrix',
    'matrix_to_pose',
    'filter_grasp_poses',
    # Camera utilities
    'CameraParameterManager',
    'get_camera_intrinsics_from_topic',
    'get_tf_transform',
    'DEFAULT_IR_K',
    'DEFAULT_RGB_K',
    'DEFAULT_T_IR_TO_RGB',
    'DEFAULT_IR_RESOLUTION',
    'DEFAULT_RGB_RESOLUTION',
    'DEFAULT_BASELINE',
]
