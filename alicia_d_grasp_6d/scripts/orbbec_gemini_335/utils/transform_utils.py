"""
Coordinate transformation utilities for 6D grasp pipeline.

This module provides functions for:
- Loading hand-eye calibration
- Transforming poses between frames
- Aligning gripper conventions
- Filtering grasp poses
"""

import os
import numpy as np
import yaml
from typing import List, Tuple, Optional, Dict, Any
from scipy.spatial.transform import Rotation as R


def load_hand_eye_calibration(calibration_path: str) -> np.ndarray:
    """
    Load hand-eye calibration transform from YAML file.
    
    Args:
        calibration_path: Path to hand_eye_calibration_result.yaml
        
    Returns:
        T_gripper_to_cam: 4x4 homogeneous transform (gripper_center -> camera_link)
    """
    if not os.path.exists(calibration_path):
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")
    
    with open(calibration_path, 'r') as f:
        calib_data = yaml.safe_load(f)
    
    hand_eye = calib_data['hand_eye_calibration']
    transform = hand_eye['transform']
    
    # Extract translation
    t = transform['translation']
    translation = np.array([t['x'], t['y'], t['z']])
    
    # Extract rotation (quaternion: x, y, z, w)
    q = transform['rotation']['quaternion']
    rotation = R.from_quat([q['x'], q['y'], q['z'], q['w']])
    
    # Build 4x4 transform matrix
    T = np.eye(4)
    T[:3, :3] = rotation.as_matrix()
    T[:3, 3] = translation
    
    return T


def pose_to_matrix(position: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    """
    Convert position and quaternion orientation to 4x4 homogeneous matrix.
    
    Args:
        position: [x, y, z] position
        orientation: [x, y, z, w] quaternion
        
    Returns:
        T: 4x4 homogeneous transform matrix
    """
    T = np.eye(4)
    T[:3, 3] = position
    T[:3, :3] = R.from_quat(orientation).as_matrix()
    return T


def matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert 4x4 homogeneous matrix to position and quaternion.
    
    Args:
        T: 4x4 homogeneous transform matrix
        
    Returns:
        position: [x, y, z] position
        orientation: [x, y, z, w] quaternion
    """
    position = T[:3, 3]
    orientation = R.from_matrix(T[:3, :3]).as_quat()
    return position, orientation


def transform_pose_to_base(grasp_cam: np.ndarray,
                           T_base_to_gripper: np.ndarray,
                           T_gripper_to_cam: np.ndarray) -> np.ndarray:
    """
    Transform grasp pose from camera frame to robot base frame.
    
    Args:
        grasp_cam: Grasp pose in camera frame (4x4)
        T_base_to_gripper: Transform from base to gripper (4x4)
        T_gripper_to_cam: Transform from gripper to camera (4x4)
        
    Returns:
        grasp_base: Grasp pose in base frame (4x4)
    """
    # Camera to base transform: T_base_cam = T_base_gripper @ T_gripper_cam
    T_base_to_cam = T_base_to_gripper @ T_gripper_to_cam
    
    # Transform grasp from camera to base frame
    grasp_base = T_base_to_cam @ grasp_cam
    
    return grasp_base


def align_gripper_convention(grasp_pose: np.ndarray,
                             source_open_axis: str = 'x',
                             target_open_axis: str = 'y') -> np.ndarray:
    """
    Align gripper coordinate convention.
    
    GraspGen uses X as finger opening direction, but actual gripper may use Y.
    This function rotates the grasp pose to align conventions.
    
    Args:
        grasp_pose: Grasp pose (4x4 matrix)
        source_open_axis: Source gripper opening axis ('x', 'y', or 'z')
        target_open_axis: Target gripper opening axis ('x', 'y', or 'z')
        
    Returns:
        aligned_pose: Aligned grasp pose (4x4 matrix)
    """
    if source_open_axis == target_open_axis:
        return grasp_pose.copy()
    
    aligned = grasp_pose.copy()
    
    # Determine rotation needed
    # From X to Y: rotate -90 degrees around Z
    # From Y to X: rotate +90 degrees around Z
    # From X to Z: rotate +90 degrees around Y
    # etc.
    
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    src_idx = axis_map[source_open_axis.lower()]
    tgt_idx = axis_map[target_open_axis.lower()]
    
    if src_idx == 0 and tgt_idx == 1:  # X -> Y
        R_align = R.from_euler('z', -90, degrees=True).as_matrix()
    elif src_idx == 1 and tgt_idx == 0:  # Y -> X
        R_align = R.from_euler('z', 90, degrees=True).as_matrix()
    elif src_idx == 0 and tgt_idx == 2:  # X -> Z
        R_align = R.from_euler('y', 90, degrees=True).as_matrix()
    elif src_idx == 2 and tgt_idx == 0:  # Z -> X
        R_align = R.from_euler('y', -90, degrees=True).as_matrix()
    elif src_idx == 1 and tgt_idx == 2:  # Y -> Z
        R_align = R.from_euler('x', -90, degrees=True).as_matrix()
    elif src_idx == 2 and tgt_idx == 1:  # Z -> Y
        R_align = R.from_euler('x', 90, degrees=True).as_matrix()
    else:
        R_align = np.eye(3)
    
    aligned[:3, :3] = aligned[:3, :3] @ R_align
    
    return aligned


def apply_gripper_offset(grasp_pose: np.ndarray,
                         offset_distance: float,
                         approach_axis: str = 'z',
                         direction: str = 'backward') -> np.ndarray:
    """
    Apply gripper depth offset to grasp pose.
    
    GraspGen outputs gripper base center pose, but hand-eye calibration
    may reference fingertip. This function adjusts the position.
    
    Args:
        grasp_pose: Grasp pose (4x4 matrix)
        offset_distance: Distance to offset (meters)
        approach_axis: Approach direction axis ('x', 'y', or 'z')
        direction: 'forward' (add offset) or 'backward' (subtract offset)
        
    Returns:
        offset_pose: Grasp pose with offset applied (4x4 matrix)
    """
    offset_pose = grasp_pose.copy()
    
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_idx = axis_map[approach_axis.lower()]
    
    # Get approach direction vector
    approach_dir = offset_pose[:3, axis_idx]
    
    # Apply offset
    if direction == 'backward':
        offset_pose[:3, 3] -= approach_dir * offset_distance
    else:
        offset_pose[:3, 3] += approach_dir * offset_distance
    
    return offset_pose


def filter_grasp_poses(grasp_poses: List[np.ndarray],
                       confidences: List[float],
                       min_z: float = 0.01,
                       max_z: float = 1.0,
                       allow_bottom_up: bool = False,
                       approach_axis: str = 'z') -> List[Tuple[int, np.ndarray, float]]:
    """
    Filter grasp poses based on validity criteria.
    
    Args:
        grasp_poses: List of grasp poses (4x4 matrices) in base frame
        confidences: List of confidence scores
        min_z: Minimum Z height (meters)
        max_z: Maximum Z height (meters)
        allow_bottom_up: Whether to allow bottom-up grasps
        approach_axis: Approach direction axis for filtering
        
    Returns:
        List of (original_index, pose, score) tuples, sorted by score
    """
    valid_grasps = []
    
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_idx = axis_map[approach_axis.lower()]
    
    for i, (grasp, conf) in enumerate(zip(grasp_poses, confidences)):
        z = grasp[2, 3]  # Height in base frame
        
        # Z height filter
        if z < min_z or z > max_z:
            continue
        
        # Bottom-up filter
        if not allow_bottom_up:
            approach_z = grasp[2, axis_idx]  # Z component of approach vector
            if approach_z > 0:  # Approach vector pointing up = bottom-up grasp
                continue
        
        valid_grasps.append((i, grasp, conf))
    
    # Sort by confidence (descending)
    valid_grasps.sort(key=lambda x: x[2], reverse=True)
    
    return valid_grasps


def compute_pre_grasp_pose(grasp_pose: np.ndarray,
                           approach_distance: float = 0.08,
                           approach_axis: str = 'z') -> np.ndarray:
    """
    Compute pre-grasp pose by retracting along approach direction.
    
    Args:
        grasp_pose: Target grasp pose (4x4 matrix)
        approach_distance: Distance to retract (meters)
        approach_axis: Approach direction axis
        
    Returns:
        pre_grasp_pose: Pre-grasp pose (4x4 matrix)
    """
    pre_grasp = grasp_pose.copy()
    
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_idx = axis_map[approach_axis.lower()]
    
    approach_dir = grasp_pose[:3, axis_idx]
    pre_grasp[:3, 3] -= approach_dir * approach_distance
    
    return pre_grasp


def compute_lift_pose(grasp_pose: np.ndarray,
                      lift_height: float = 0.10) -> np.ndarray:
    """
    Compute lift pose by raising in Z direction.
    
    Args:
        grasp_pose: Grasp pose (4x4 matrix)
        lift_height: Height to lift (meters)
        
    Returns:
        lift_pose: Lifted pose (4x4 matrix)
    """
    lift_pose = grasp_pose.copy()
    lift_pose[2, 3] += lift_height
    return lift_pose


def depth2xyzmap(depth: np.ndarray, K: np.ndarray,
                 z_min: float = 0.01) -> np.ndarray:
    """
    Convert depth image to XYZ point cloud map.
    
    Args:
        depth: Depth image (H, W) in meters
        K: Camera intrinsic matrix (3x3)
        z_min: Minimum valid depth
        
    Returns:
        xyz_map: XYZ point cloud (H, W, 3)
    """
    H, W = depth.shape[:2]
    
    # Create pixel coordinate grids
    vs, us = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
    
    # Get depth values
    zs = depth.copy()
    invalid_mask = zs < z_min
    
    # Back-project to 3D
    xs = (us - K[0, 2]) * zs / K[0, 0]
    ys = (vs - K[1, 2]) * zs / K[1, 1]
    
    # Stack into XYZ map
    xyz_map = np.stack([xs, ys, zs], axis=-1).astype(np.float32)
    
    # Mark invalid points
    xyz_map[invalid_mask] = 0
    
    return xyz_map


def project_points_to_image(points: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Project 3D points to 2D image coordinates.
    
    Args:
        points: 3D points (N, 3)
        K: Camera intrinsic matrix (3x3)
        
    Returns:
        uv: 2D image coordinates (N, 2)
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    # Avoid division by zero
    z = np.clip(z, 0.001, None)
    
    u = fx * x / z + cx
    v = fy * y / z + cy
    
    return np.stack([u, v], axis=1)


def extract_masked_points(pointcloud: np.ndarray,
                          mask: np.ndarray,
                          K: np.ndarray) -> np.ndarray:
    """
    Extract 3D points that correspond to a 2D mask.
    
    Projects point cloud to 2D and keeps points within mask.
    
    Args:
        pointcloud: Full scene point cloud (N, 3)
        mask: 2D binary mask (H, W)
        K: Camera intrinsic matrix (3x3)
        
    Returns:
        object_points: Points within mask (M, 3)
    """
    H, W = mask.shape
    
    # Project points to image plane
    uv = project_points_to_image(pointcloud, K)
    
    # Check which points fall within mask
    valid_indices = []
    for i, (u, v) in enumerate(uv):
        if 0 <= int(u) < W and 0 <= int(v) < H:
            if mask[int(v), int(u)]:
                valid_indices.append(i)
    
    if len(valid_indices) == 0:
        return np.array([]).reshape(0, 3)
    
    return pointcloud[valid_indices]


class GraspPoseTransformer:
    """
    Class for transforming and processing grasp poses.
    
    Encapsulates the full transformation pipeline:
    1. Camera frame to base frame
    2. Gripper convention alignment
    3. Gripper offset correction
    """
    
    def __init__(self,
                 T_gripper_to_cam: np.ndarray,
                 gripper_depth: float = 0.13,
                 source_open_axis: str = 'x',
                 target_open_axis: str = 'y'):
        """
        Initialize the transformer.
        
        Args:
            T_gripper_to_cam: Hand-eye calibration transform (4x4)
            gripper_depth: Distance from gripper base center to fingertip (m)
            source_open_axis: GraspGen gripper opening axis
            target_open_axis: Actual gripper opening axis
        """
        self.T_gripper_to_cam = T_gripper_to_cam
        self.gripper_depth = gripper_depth
        self.source_open_axis = source_open_axis
        self.target_open_axis = target_open_axis
    
    def transform(self, grasp_cam: np.ndarray,
                  T_base_to_gripper: np.ndarray) -> np.ndarray:
        """
        Transform grasp from camera frame to base frame with all corrections.
        
        Args:
            grasp_cam: Grasp pose in camera frame (4x4)
            T_base_to_gripper: Current gripper pose in base frame (4x4)
            
        Returns:
            grasp_base: Corrected grasp pose in base frame (4x4)
        """
        # Step 1: Transform to base frame
        grasp_base = transform_pose_to_base(
            grasp_cam, T_base_to_gripper, self.T_gripper_to_cam)
        
        # Step 2: Align gripper convention
        grasp_base = align_gripper_convention(
            grasp_base, self.source_open_axis, self.target_open_axis)
        
        # Step 3: Apply gripper offset
        grasp_base = apply_gripper_offset(
            grasp_base, self.gripper_depth, 'z', 'backward')
        
        return grasp_base
