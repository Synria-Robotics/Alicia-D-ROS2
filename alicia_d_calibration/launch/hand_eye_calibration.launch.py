"""Launch file for hand-eye calibration with Alicia-D robot and camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for hand-eye calibration."""
    
    # 声明启动参数
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_4X4_50',
        description='ArUco dictionary type (e.g., DICT_4X4_50, DICT_5X5_100)'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='ArUco marker size in meters'
    )
    
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='Target ArUco marker ID'
    )
    
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/image_raw
    # 若使用RealSense D405，则使用/camera/camera/color/image_rect_raw
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/camera/color/image_rect_raw',
        description='Camera image topic'
    )
    
    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/camera_info
    # 若使用RealSense D405，则使用/camera/camera/color/camera_info
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Camera info topic'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Robot base link name'
    )
    
    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='gripper_center',
        description='Robot end effector link name'
    )

    calibration_type_arg = DeclareLaunchArgument(
        'calibration_type',
        default_value='eye_in_hand',
        description='Calibration type: eye_in_hand or eye_to_hand'
    )
    
    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value='17',
        description='Minimum number of calibration samples'
    )
    
    calibration_method_arg = DeclareLaunchArgument(
        'calibration_method',
        default_value='daniilidis',
        description='Hand-eye calibration method (tsai, park, horaud, daniilidis, andreff, auto)'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='hand_eye_calibration_result.yaml',
        description='Output file name for calibration result'
    )
    
    marker_distance_arg = DeclareLaunchArgument(
        'marker_distance',
        default_value='0.32',
        description='Distance from ArUco marker to base_link in meters'
    )

    eye_to_hand_joint5_center_arg = DeclareLaunchArgument(
        'eye_to_hand_joint5_center',
        default_value='0.85',
        description='Eye-to-hand: Joint5 center angle for marker facing camera (rad)'
    )

    eye_to_hand_joint5_span_arg = DeclareLaunchArgument(
        'eye_to_hand_joint5_span',
        default_value='0.35',
        description='Eye-to-hand: Joint5 perturbation range around center (rad)'
    )

    eye_to_hand_joint2_offset_arg = DeclareLaunchArgument(
        'eye_to_hand_joint2_offset',
        default_value='0.0',
        description='Eye-to-hand: Joint2 offset (rad), negative lowers end-effector'
    )

    eye_to_hand_joint3_offset_arg = DeclareLaunchArgument(
        'eye_to_hand_joint3_offset',
        default_value='0.0',
        description='Eye-to-hand: Joint3 offset (rad), negative lowers end-effector'
    )
    
    # 手眼标定节点
    calibration_node = Node(
        package='alicia_d_calibration',
        executable='hand_eye_calibration.py',
        name='hand_eye_calibration',
        output='screen',
        parameters=[{
            'aruco_dict': LaunchConfiguration('aruco_dict'),
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'base_link': LaunchConfiguration('base_link'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'min_samples': LaunchConfiguration('min_samples'),
            'calibration_method': LaunchConfiguration('calibration_method'),
            'output_file': LaunchConfiguration('output_file'),
            'marker_distance': LaunchConfiguration('marker_distance'),
            'eye_to_hand_joint5_center': LaunchConfiguration('eye_to_hand_joint5_center'),
            'eye_to_hand_joint5_span': LaunchConfiguration('eye_to_hand_joint5_span'),
            'eye_to_hand_joint2_offset': LaunchConfiguration('eye_to_hand_joint2_offset'),
            'eye_to_hand_joint3_offset': LaunchConfiguration('eye_to_hand_joint3_offset'),
        }],
    )
    
    return LaunchDescription([
        # 日志信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Alicia-D Hand-Eye Calibration'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Make sure the robot and camera are started:'),
        LogInfo(msg='  1. ros2 launch alicia_d_moveit real_robot.launch.py'),
        LogInfo(msg='  2. ros2 launch orbbec_camera gemini_335.launch.py'),
        LogInfo(msg='========================================'),
        
        # 参数声明
        aruco_dict_arg,
        marker_size_arg,
        marker_id_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        base_link_arg,
        end_effector_link_arg,
        calibration_type_arg,
        min_samples_arg,
        calibration_method_arg,
        output_file_arg,
        marker_distance_arg,
        eye_to_hand_joint5_center_arg,
        eye_to_hand_joint5_span_arg,
        eye_to_hand_joint2_offset_arg,
        eye_to_hand_joint3_offset_arg,
        
        # 节点
        calibration_node,
    ])
