import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener


def _parse_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def get_tf_as_matrix(source_frame: str, target_frame: str, timeout_sec: float = 5.0):
    """
    从 TF 读取 source_frame -> target_frame 的变换，并返回 4x4 齐次变换矩阵。
    """
    if not rclpy.ok():
        rclpy.init()

    node = RclpyNode('_tf_lookup_temp_node')
    tf_buffer = Buffer()
    _ = TransformListener(tf_buffer, node)

    try:
        start_time = time.time()
        transform = None

        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time(),
                )
                break
            except Exception:
                pass

        if transform is None:
            print(f"[警告] 无法在 {timeout_sec} 秒内获取 TF: {source_frame} -> {target_frame}")
            return None

        t = transform.transform.translation
        q = transform.transform.rotation

        r_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        m = np.eye(4)
        m[:3, :3] = r_mat
        m[:3, 3] = [t.x, t.y, t.z]

        print(f"[TF] 成功读取 {source_frame} -> {target_frame}")
        print(f"     平移: [{t.x:.6f}, {t.y:.6f}, {t.z:.6f}]")
        print(f"     四元数: [{q.x:.6f}, {q.y:.6f}, {q.z:.6f}, {q.w:.6f}]")

        return m
    finally:
        node.destroy_node()


def _resolve_calibration_path(calibration_file: str) -> str:
    if os.path.isabs(calibration_file):
        return calibration_file

    # Try common locations (installed share, source tree, nested workspace).
    candidates = []

    # Relative to this launch file: <pkg>/config/<file>
    try:
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.abspath(os.path.join(this_dir, '..', 'config', calibration_file)))
    except Exception:
        pass

    # Installed share: <share>/config/<file>
    try:
        share_dir = Path(get_package_share_directory('alicia_d_calibration')).resolve()
        candidates.append(str((share_dir / 'config' / calibration_file).resolve()))

        # Workspace root guesses: <root>/alicia_d_calibration/config or <root>/src/alicia_d_calibration/config
        # share_dir looks like: <root>/install/alicia_d_calibration/share/alicia_d_calibration
        for parent in list(share_dir.parents)[:8]:
            candidates.append(str((parent / 'alicia_d_calibration' / 'config' / calibration_file).resolve()))
            candidates.append(str((parent / 'src' / 'alicia_d_calibration' / 'config' / calibration_file).resolve()))
    except Exception:
        pass

    for p in candidates:
        if p and os.path.exists(p):
            return p

    return calibration_file


def _load_calibration_matrix(hand_eye: dict):
    transform = hand_eye['transform']
    trans_data = transform['translation']
    rot_data = transform['rotation']['quaternion']

    t = np.array([trans_data['x'], trans_data['y'], trans_data['z']])
    r = R.from_quat([rot_data['x'], rot_data['y'], rot_data['z'], rot_data['w']])

    m = np.eye(4)
    m[:3, :3] = r.as_matrix()
    m[:3, 3] = t
    return m


def load_calibration_result(context, *args, **kwargs):
    """加载标定结果，构建静态 TF，并启动 ArUco 验证节点。"""
    calibration_file = LaunchConfiguration('calibration_file').perform(context)
    aruco_dict = LaunchConfiguration('aruco_dict').perform(context)
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    camera_optical_frame = LaunchConfiguration('camera_optical_frame').perform(context)
    marker_frame = LaunchConfiguration('marker_frame').perform(context)
    base_link = LaunchConfiguration('base_link').perform(context)
    end_effector_link = LaunchConfiguration('end_effector_link').perform(context)
    sample_rate_hz = float(LaunchConfiguration('verify_sample_rate_hz').perform(context))
    window_size = int(LaunchConfiguration('verify_window_size').perform(context))
    min_samples = int(LaunchConfiguration('verify_min_samples').perform(context))
    report_period_sec = float(LaunchConfiguration('verify_report_period_sec').perform(context))
    estimate_hand_eye = _parse_bool(LaunchConfiguration('estimate_hand_eye').perform(context))
    estimate_method = LaunchConfiguration('estimate_method').perform(context)
    estimate_output_file = LaunchConfiguration('estimate_output_file').perform(context)
    apply_optical_correction = _parse_bool(
        LaunchConfiguration('apply_optical_correction').perform(context)
    )

    calibration_file = _resolve_calibration_path(calibration_file)
    if not os.path.exists(calibration_file):
        print(f"错误: 找不到标定文件: {calibration_file}")
        return []

    with open(calibration_file, 'r') as f:
        calib_data = yaml.safe_load(f)

    hand_eye = calib_data['hand_eye_calibration']
    calibration_type = hand_eye.get('type', 'eye_in_hand')
    parent_frame = hand_eye.get('frame_id', 'gripper_center')
    child_frame = hand_eye.get('child_frame_id', 'camera_link')

    print('\n[校验] 读取到标定结果:')
    print(f"  type: {calibration_type}")
    print(f"  frame_id: {parent_frame}")
    print(f"  child_frame_id: {child_frame}")

    # YAML 中结果来自 ArUco/PnP 的相机光学系，需转换到 camera_link 再发布。
    # NOTE:
    # - `camera_optical_frame` is the frame used by ArUco/PnP (typically camera_color_optical_frame).
    # - `child_frame_id` in YAML is typically camera_link.
    # - If `apply_optical_correction=true`, we use the camera-internal TF to convert
    #   the YAML matrix from optical frame into `child_frame_id`.
    m_calib = _load_calibration_matrix(hand_eye)
    m_final = m_calib

    if apply_optical_correction and child_frame != camera_optical_frame:
        print('\n[修正] 将标定结果从相机光学系转换到 camera_link...')
        print(f"  读取 TF: {child_frame} -> {camera_optical_frame}")

        m_link_to_optical = get_tf_as_matrix(
            child_frame,
            camera_optical_frame,
            timeout_sec=5.0,
        )

        if m_link_to_optical is None:
            print('[错误] 无法获取相机内部 TF，无法完成 Optical -> Link 修正')
            print('      请确认相机驱动已启动并发布相应 TF')
            return []

        # parent->link = parent->optical * optical->link
        m_final = m_calib @ np.linalg.inv(m_link_to_optical)

    final_t = m_final[:3, 3]
    final_q = R.from_matrix(m_final[:3, :3]).as_quat()

    print(f"\n[发布] 静态 TF: {parent_frame} -> {child_frame}")
    print(f"  平移: [{final_t[0]:.6f}, {final_t[1]:.6f}, {final_t[2]:.6f}]")
    print(f"  四元数: [{final_q[0]:.6f}, {final_q[1]:.6f}, {final_q[2]:.6f}, {final_q[3]:.6f}]")

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_calibration_publisher',
        arguments=[
            str(final_t[0]),
            str(final_t[1]),
            str(final_t[2]),
            str(final_q[0]),
            str(final_q[1]),
            str(final_q[2]),
            str(final_q[3]),
            parent_frame,
            child_frame,
        ],
        output='screen',
    )

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
            'camera_frame': camera_optical_frame,
            'marker_frame': marker_frame,
        }],
    )

    verifier_node = Node(
        package='alicia_d_calibration',
        executable='calibration_verifier.py',
        name='calibration_verifier',
        output='screen',
        parameters=[{
            'calibration_type': calibration_type,
            'base_link': base_link,
            'end_effector_link': end_effector_link,
            'marker_frame': marker_frame,
            # For eye_to_hand estimation, verifier needs to know the camera optical frame
            # and the camera link frame (the YAML child frame).
            'camera_optical_frame': camera_optical_frame,
            'camera_link_frame': child_frame,
            'sample_rate_hz': sample_rate_hz,
            'window_size': window_size,
            'min_samples': min_samples,
            'report_period_sec': report_period_sec,
            # Optional: estimate and print `base_link <- camera_link` matrix from TF samples (eye_to_hand).
            'estimate_hand_eye': estimate_hand_eye,
            'estimate_method': estimate_method,
            'estimate_output_file': estimate_output_file,
        }],
    )

    return [static_tf_node, aruco_detector_node, verifier_node]


def generate_launch_description():
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='hand_eye_calibration_result.yaml',
    )
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_4X4_50',
    )

    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/image_raw
    # 若使用RealSense D405，则使用/camera/camera/color/image_rect_raw
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/camera/color/image_rect_raw',
    )

    # 根据相机实际话题修改
    # 若使用Gemini 335，则使用/camera/color/camera_info
    # 若使用RealSense D405，则使用/camera/camera/color/camera_info
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
    )

    camera_optical_frame_arg = DeclareLaunchArgument(
        'camera_optical_frame',
        default_value='camera_color_optical_frame',
        description='Aruco/PnP 使用的相机光学坐标系',
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame',
        default_value='aruco_marker_frame',
        description='验证时发布的 ArUco 标记坐标系',
    )

    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Robot base link (used for eye_in_hand verification)',
    )

    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='gripper_center',
        description='Robot end-effector link (used for eye_to_hand verification)',
    )

    verify_sample_rate_hz_arg = DeclareLaunchArgument(
        'verify_sample_rate_hz',
        default_value='10.0',
        description='Verifier sampling rate (Hz)',
    )

    verify_window_size_arg = DeclareLaunchArgument(
        'verify_window_size',
        default_value='100',
        description='Verifier sliding window size',
    )

    verify_min_samples_arg = DeclareLaunchArgument(
        'verify_min_samples',
        default_value='30',
        description='Minimum samples before reporting',
    )

    verify_report_period_sec_arg = DeclareLaunchArgument(
        'verify_report_period_sec',
        default_value='2.0',
        description='Report period (seconds)',
    )

    apply_optical_correction_arg = DeclareLaunchArgument(
        'apply_optical_correction',
        default_value='true',
        description='是否将 YAML 中光学系结果转换到 child_frame_id',
    )

    estimate_hand_eye_arg = DeclareLaunchArgument(
        'estimate_hand_eye',
        default_value='true',
        description='是否在校验时从 TF 采样估计 base<-camera 矩阵（仅 eye_to_hand 有效）',
    )

    estimate_method_arg = DeclareLaunchArgument(
        'estimate_method',
        default_value='daniilidis',
        description='估计/校验时使用的 OpenCV 手眼算法 (tsai, park, horaud, daniilidis, andreff)',
    )

    estimate_output_file_arg = DeclareLaunchArgument(
        'estimate_output_file',
        default_value='',
        description='可选：将估计得到的 base<-camera 写入该 YAML 文件路径（空=不写）',
    )

    return LaunchDescription([
        calibration_file_arg,
        aruco_dict_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        camera_optical_frame_arg,
        marker_frame_arg,
        base_link_arg,
        end_effector_link_arg,
        verify_sample_rate_hz_arg,
        verify_window_size_arg,
        verify_min_samples_arg,
        verify_report_period_sec_arg,
        apply_optical_correction_arg,
        estimate_hand_eye_arg,
        estimate_method_arg,
        estimate_output_file_arg,
        OpaqueFunction(function=load_calibration_result),
    ])
