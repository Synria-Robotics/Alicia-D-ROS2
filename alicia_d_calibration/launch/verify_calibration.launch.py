# #!/usr/bin/env python3

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node as RclpyNode
from tf2_ros import Buffer, TransformListener
import time


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
    # 初始化 rclpy（如果尚未初始化）
    if not rclpy.ok():
        rclpy.init()
    
    # 创建临时节点用于 TF 查询
    node = RclpyNode('_tf_lookup_temp_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    try:
        # 等待 TF 可用
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
            print(f"[警告] 无法在 {timeout_sec} 秒内获取 TF: {source_frame} -> {target_frame}")
            return None
        
        # 提取平移和旋转
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # 构建 4x4 矩阵
        r_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        m = np.eye(4)
        m[:3, :3] = r_mat
        m[:3, 3] = [t.x, t.y, t.z]
        
        print(f"[TF] 成功读取 {source_frame} -> {target_frame}:")
        print(f"     平移: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
        print(f"     四元数: [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]")
        
        return m
        
    finally:
        node.destroy_node()


def load_calibration_result(context, *args, **kwargs):
    """加载标定结果，进行坐标系转换，并创建节点"""
    
    # 获取参数
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    aruco_dict = LaunchConfiguration('aruco_dict').perform(context)
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    
    # 路径处理
    if not os.path.isabs(calibration_file):
        try:
            package_share_dir = get_package_share_directory('alicia_d_calibration')
            workspace_root = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..'))
            calibration_file = os.path.join(workspace_root, 'src', 'alicia_d_calibration', 'config', calibration_file)
        except Exception:
            pass
    
    if not os.path.exists(calibration_file):
        print(f"错误: 找不到标定文件: {calibration_file}")
        return []
    
    with open(calibration_file, 'r') as f:
        calib_data = yaml.safe_load(f)
    
    hand_eye = calib_data['hand_eye_calibration']
    transform = hand_eye['transform']
    trans_data = transform['translation']
    rot_data = transform['rotation']['quaternion']
    
    # ====================================================================
    # 核心修正逻辑：Optical Frame -> Link Frame 转换
    # ====================================================================
    print("\n[数学修正] 正在将 Optical 标定数据转换为 Link 坐标系...")

    # 1. 构建标定矩阵 T_gripper_optical (从 YAML 读取)
    t_calib = np.array([trans_data['x'], trans_data['y'], trans_data['z']])
    r_calib = R.from_quat([rot_data['x'], rot_data['y'], rot_data['z'], rot_data['w']])
    
    m_calib = np.eye(4)
    m_calib[:3, :3] = r_calib.as_matrix()
    m_calib[:3, 3] = t_calib

    # 2. 从 TF 读取相机内部矩阵 T_link_optical (camera_link -> camera_color_optical_frame)
    print("\n[TF] 正在从 TF 读取 camera_link -> camera_color_optical_frame 变换...")
    m_internal = get_tf_as_matrix('camera_link', 'camera_color_optical_frame', timeout_sec=5.0)
    
    if m_internal is None:
        print("[错误] 无法从 TF 获取 camera_link -> camera_color_optical_frame 变换")
        print("       请确保相机节点已启动并发布 TF")
        return []

    # 3. 计算最终变换 T_gripper_link = T_gripper_optical * inv(T_link_optical)
    m_final = m_calib @ np.linalg.inv(m_internal)

    # 4. 提取修正后的位姿
    final_t = m_final[:3, 3]
    final_q = R.from_matrix(m_final[:3, :3]).as_quat()

    print(f"  原始(Optical): t={t_calib}")
    print(f"  修正(Link)   : t={final_t}")
    # ====================================================================

    # 强制指定坐标系
    parent_frame = hand_eye.get('frame_id', 'gripper_center')
    child_frame = 'camera_link' # 必须发布到 camera_link 以维持 TF 树连通性

    # 1. 静态TF发布器 (使用修正后的数据)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_calibration_publisher',
        arguments=[
            str(final_t[0]), str(final_t[1]), str(final_t[2]),
            str(final_q[0]), str(final_q[1]), str(final_q[2]), str(final_q[3]),
            parent_frame,
            child_frame
        ],
        output='screen'
    )
    
    # 2. ArUco 检测节点 (必须告诉它去听 Optical Frame)
    aruco_detector_node = Node(
        package='alicia_d_calibration',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'aruco_dict': aruco_dict,
            'marker_size': hand_eye.get('aruco_marker_size', 0.05),
            'marker_id': hand_eye.get('aruco_marker_id', 0),
            'camera_topic': camera_topic,
            'camera_info_topic': camera_info_topic,
            'camera_frame': 'camera_color_optical_frame', # 关键！检测基于光学系
            'marker_frame': 'aruco_marker_frame'
        }]
    )
    
    return [static_tf_node, aruco_detector_node]


def generate_launch_description():
    """生成 Launch 描述"""
    calibration_file_arg = DeclareLaunchArgument('calibration_file', default_value='hand_eye_calibration_result.yaml')
    aruco_dict_arg = DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50')
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/image_raw
    # 若使用RealSense D405，则使用/camera/camera/color/image_rect_raw
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/camera/camera/color/image_rect_raw')
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/camera_info
    # 若使用RealSense D405，则使用/camera/camera/color/camera_info
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info')
    
    return LaunchDescription([
        calibration_file_arg,
        aruco_dict_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        OpaqueFunction(function=load_calibration_result)
    ])