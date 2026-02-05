# #!/usr/bin/env python3
# """
# 手眼标定结果验证 Launch 文件

# 使用方法:
# 1. 启动机械臂:
#    ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm

# 2. 启动相机:
#    Gemini 335:
#    ros2 launch orbbec_camera gemini_335.launch.py
#    RealSense D405:
#    ros2 launch realsense2_camera rs_launch.py


# 3. 启动此验证文件:
#    ros2 launch alicia_d_calibration verify_calibration.launch.py

# 4. 查看 TF 树:
#    ros2 run rqt_tf_tree rqt_tf_tree --force-discover 

# 5. 验证标定结果 (将标记放置在标定时的位置):
#    ros2 run tf2_ros tf2_echo base_link aruco_marker_frame
# """

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
            calibration_file = os.path.join(workspace_root, 'src', 'alicia_d_calibation', 'config', calibration_file)
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

    # 2. 构建相机内部矩阵 T_link_optical
    
    # m_internal = np.array([
    #     [ 0.0, -0.0,  1.0, 0.002],
    #     [-1.0, -0.0, -0.0, -0.014],
    #     [ 0.0, -1.0,  0.0, 0.000],
    #     [ 0.0,  0.0,  0.0, 1.0]
    # ])
    m_internal = np.array([
        [ -0.001, -0.001,  1.000, 0.000],
        [ -1.000, 0.001, -0.001, -0.000],
        [ -0.001, -1.000,  -0.001, 0.000],
        [ 0.000,  0.000,  0.000, 1.000]
    ])

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