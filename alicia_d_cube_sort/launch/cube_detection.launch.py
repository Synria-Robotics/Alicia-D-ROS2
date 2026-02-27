#!/usr/bin/env python3
"""
Cube Detection Launch File

This launch file starts the cube detection node along with the hand-eye 
calibration TF publisher. It loads camera intrinsics/extrinsics and publishes
the static TF between gripper_center and camera_link.
"""

import os
import yaml
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node as RclpyNode
from tf2_ros import Buffer, TransformListener


def get_tf_as_matrix(source_frame: str, target_frame: str, timeout_sec: float = 5.0) -> np.ndarray:
    """
    从 TF 读取 source_frame -> target_frame 的变换，并返回 4x4 齐次变换矩阵。
    
    Args:
        source_frame: 源坐标系
        target_frame: 目标坐标系
        timeout_sec: 等待 TF 的超时时间
        
    Returns:
        4x4 齐次变换矩阵，如果失败返回 None
    """
    if not rclpy.ok():
        rclpy.init()
    
    node = RclpyNode('_tf_lookup_temp_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    try:
        start_time = time.time()
        transform = None
        
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = tf_buffer.lookup_transform(
                    source_frame, target_frame, rclpy.time.Time())
                break
            except Exception:
                pass
        
        if transform is None:
            print(f"[WARNING] Cannot get TF: {source_frame} -> {target_frame} within {timeout_sec}s")
            return None
        
        t = transform.transform.translation
        q = transform.transform.rotation
        
        r_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        m = np.eye(4)
        m[:3, :3] = r_mat
        m[:3, 3] = [t.x, t.y, t.z]
        
        print(f"[TF] Read {source_frame} -> {target_frame}:")
        print(f"     Translation: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
        print(f"     Quaternion: [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]")
        
        return m
        
    finally:
        node.destroy_node()


def load_calibration_and_create_nodes(context, *args, **kwargs):
    """Load calibration data and create nodes"""
    
    # Get launch arguments
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    depth_mode = LaunchConfiguration('depth_mode').perform(context)
    show_image = LaunchConfiguration('show_image').perform(context)
    color_topic = LaunchConfiguration('color_topic').perform(context)
    color_info_topic = LaunchConfiguration('color_info_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    
    # Find calibration file
    calibration_path = calibration_file
    if not os.path.isabs(calibration_path):
        try:
            pkg_share = get_package_share_directory('alicia_d_calibration')
            workspace_root = os.path.abspath(
                os.path.join(pkg_share, '..', '..', '..', '..'))
            calibration_path = os.path.join(
                workspace_root, 'src', 'alicia_d_calibration', 'config', 
                calibration_file)
        except Exception:
            pass
    
    nodes = []
    
    # Load calibration and publish static TF
    if os.path.exists(calibration_path):
        try:
            with open(calibration_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            hand_eye = calib_data.get('hand_eye_calibration', {})
            transform = hand_eye.get('transform', {})
            trans = transform.get('translation', {})
            rot = transform.get('rotation', {}).get('quaternion', {})
            
            # Build calibration matrix
            t_calib = np.array([trans.get('x', 0), trans.get('y', 0), trans.get('z', 0)])
            r_calib = R.from_quat([rot.get('x', 0), rot.get('y', 0), 
                                   rot.get('z', 0), rot.get('w', 1)])
            
            m_calib = np.eye(4)
            m_calib[:3, :3] = r_calib.as_matrix()
            m_calib[:3, 3] = t_calib
            
            # Read T_link_optical from TF (camera_link -> camera_color_optical_frame)
            print("\n[TF] Reading camera_link -> camera_color_optical_frame transform...")
            m_internal = get_tf_as_matrix('camera_link', 'camera_color_optical_frame', timeout_sec=5.0)
            
            if m_internal is None:
                print("[ERROR] Cannot get camera_link -> camera_color_optical_frame from TF")
                print("        Make sure camera node is running and publishing TF")
            else:
                # Final transform: T_gripper_link = T_gripper_optical * inv(T_link_optical)
                m_final = m_calib @ np.linalg.inv(m_internal)
                
                final_t = m_final[:3, 3]
                final_q = R.from_matrix(m_final[:3, :3]).as_quat()
                
                parent_frame = hand_eye.get('frame_id', 'gripper_center')
                child_frame = 'camera_link'
                
                print("\n" + "=" * 60)
                print("[Cube Detection] Camera Extrinsics (Hand-Eye Calibration):")
                print(f"  Parent Frame: {parent_frame}")
                print(f"  Child Frame: {child_frame}")
                print(f"  Translation: [{final_t[0]:.6f}, {final_t[1]:.6f}, {final_t[2]:.6f}]")
                print(f"  Quaternion: [{final_q[0]:.6f}, {final_q[1]:.6f}, {final_q[2]:.6f}, {final_q[3]:.6f}]")
                print("=" * 60 + "\n")
                
                # Static TF publisher
                static_tf_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='hand_eye_tf_publisher',
                    arguments=[
                        str(final_t[0]), str(final_t[1]), str(final_t[2]),
                        str(final_q[0]), str(final_q[1]), str(final_q[2]), str(final_q[3]),
                        parent_frame, child_frame
                    ],
                    output='screen'
                )
                nodes.append(static_tf_node)
            
        except Exception as e:
            print(f"[ERROR] Failed to load calibration: {e}")
    else:
        print(f"[WARNING] Calibration file not found: {calibration_path}")
    
    # Find config file
    try:
        pkg_share = get_package_share_directory('alicia_d_cube_sort')
        config_file = os.path.join(pkg_share, 'config', 'cube_sorting.yaml')
    except Exception:
        config_file = ''
    
    # Cube detection node
    cube_detector_node = Node(
        package='alicia_d_cube_sort',
        executable='cube_detection.py',
        name='cube_detector',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'depth_mode': depth_mode.lower() == 'true',
            'show_image': show_image.lower() == 'true',
            'camera_frame': 'camera_color_optical_frame',
            'base_frame': base_frame,
            'color_topic': color_topic,
            'color_info_topic': color_info_topic,
            'depth_topic': depth_topic,
            'calibration_file': calibration_file
        }]
    )
    nodes.append(cube_detector_node)
    
    return nodes


def generate_launch_description():
    """Generate launch description"""
    
    # Launch arguments
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='hand_eye_calibration_result.yaml',
        description='Hand-eye calibration result file'
    )
    
    depth_mode_arg = DeclareLaunchArgument(
        'depth_mode',
        default_value='false',
        description='Use depth camera for 3D position (true) or A4 paper plane method (false)'
    )
    
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show OpenCV visualization window'
    )
    
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/image_raw
    # 若使用RealSense D405，则使用/camera/camera/color/image_rect_raw
    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/camera/color/image_rect_raw',
        description='Color image topic'
    )
    
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/camera_info
    # 若使用RealSense D405，则使用/camera/camera/color/camera_info
    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Color camera info topic'
    )
    
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/depth/image_raw
    # 若使用RealSense D405，则使用/camera/camera/depth/image_rect_raw
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/depth/image_rect_raw',
        description='Depth image topic'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame for coordinate transformation'
    )
    
    return LaunchDescription([
        calibration_file_arg,
        depth_mode_arg,
        show_image_arg,
        color_topic_arg,
        color_info_topic_arg,
        depth_topic_arg,
        base_frame_arg,
        OpaqueFunction(function=load_calibration_and_create_nodes)
    ])
