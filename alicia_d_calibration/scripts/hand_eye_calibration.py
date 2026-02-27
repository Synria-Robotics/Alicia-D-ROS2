#!/usr/bin/env python3
"""
Alicia-D 机械臂手眼标定脚本 (Eye-in-Hand)

使用 ArUco 标记和 Gemini335 相机进行手眼标定。
标定完成后，输出机械臂末端到相机的变换矩阵，保存在../config/hand_eye_calibration_result.yaml
"""

import sys
import os
import time
import threading
import numpy as np
import cv2
import yaml
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
from rclpy.duration import Duration


class HandEyeCalibration(Node):
    """手眼标定节点 (Eye-in-Hand 配置)"""
    
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # 声明参数
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)  # 标记边长 (米)
        self.declare_parameter('marker_id', 0)  # 目标标记ID
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'gripper_center')
        self.declare_parameter('min_samples', 17)  # 最小采样数
        self.declare_parameter('calibration_method', 'tsai')  # 标定算法: tsai, park, horaud, daniilidis
        self.declare_parameter('output_file', 'hand_eye_calibration_result.yaml')
        self.declare_parameter('marker_distance', 0.32)  # ArUco标记距离base_link的距离 (米)
        
        # 获取参数
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_link = self.get_parameter('base_link').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.min_samples = self.get_parameter('min_samples').value
        self.calibration_method = self.get_parameter('calibration_method').value
        self.output_file = self.get_parameter('output_file').value
        self.marker_distance = self.get_parameter('marker_distance').value
        
        # 初始化 ArUco 检测器
        self._init_aruco_detector()
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # 当前图像和检测结果
        self.current_image = None
        self.current_rvec = None
        self.current_tvec = None
        self.marker_detected = False
        self.image_lock = threading.Lock()
        
        # ArUco检测去噪：多帧历史缓冲区
        self.detection_history_size = 10  # 保存最近10帧检测结果
        self.detection_min_frames = 5     # 至少需要5帧有效检测
        self.detection_rvec_history = []  # 旋转向量历史
        self.detection_tvec_history = []  # 平移向量历史
        self.denoised_rvec = None         # 去噪后的旋转向量
        self.denoised_tvec = None         # 去噪后的平移向量
        self.detection_stable = False     # 检测是否稳定
        
        # 采样数据存储
        self.R_gripper2base_samples = []  # 末端到基座的旋转矩阵
        self.t_gripper2base_samples = []  # 末端到基座的平移向量
        self.R_target2cam_samples = []    # 标记到相机的旋转矩阵
        self.t_target2cam_samples = []    # 标记到相机的平移向量
        
        # 机械臂控制
        self.arm_joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/Alicia_controller/follow_joint_trajectory'
        )
        
        # MoveIt ExecuteTrajectory action 客户端
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')
        
        # 运动规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._image_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10
        )
        
        # 图像显示定时器
        self.display_timer = self.create_timer(0.033, self._display_image)  # ~30 FPS
        
        # 标定位姿列表 (根据marker_distance生成)
        self.calibration_poses = self._generate_calibration_poses()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('手眼标定节点已启动 (Eye-in-Hand)')
        self.get_logger().info(f'ArUco 字典: {self.aruco_dict_name}')
        self.get_logger().info(f'标记大小: {self.marker_size * 100:.1f} cm')
        self.get_logger().info(f'目标标记ID: {self.marker_id}')
        self.get_logger().info(f'最小采样数: {self.min_samples}')
        self.get_logger().info(f'标定算法: {self.calibration_method}')
        self.get_logger().info(f'标记距离: {self.marker_distance * 100:.1f} cm')
        self.get_logger().info('=' * 60)
    
    def _init_aruco_detector(self):
        """初始化 ArUco 检测器"""
        # ArUco 字典映射
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        }
        
        if self.aruco_dict_name not in aruco_dict_map:
            self.get_logger().warn(f'未知的 ArUco 字典: {self.aruco_dict_name}, 使用默认 DICT_4X4_50')
            self.aruco_dict_name = 'DICT_4X4_50'
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[self.aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    
    def _generate_calibration_poses(self):
        """
        生成标定用的机械臂位姿
        
        考虑到ArUco标记在base_link正前方30-35cm处，需要确保标记在相机视野内
        位姿需要有足够的旋转差异性来提高标定精度
        """
        poses = []
        
        # 基础位姿 - 面向前方，确保相机能看到标记
        # 这些是关节角度 [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6] (弧度)
        
        # 位姿1: 正面直视
        poses.append([0.0, -0.3, 0.6, 0.0, -0.9, 0.0])
        
        # 位姿2-4: 左右偏转 (绕Joint1旋转)
        poses.append([-0.15, -0.3, 0.6, 0.0, -0.9, 0.0])
        poses.append([0.15, -0.3, 0.6, 0.0, -0.9, 0.0])
        poses.append([-0.25, -0.25, 0.55, 0.0, -0.85, 0.0])
        
        # 位姿5-7: 上下调整 (改变俯仰角)
        poses.append([0.0, -0.4, 0.7, 0.0, -0.85, 0.0])
        poses.append([0.0, -0.2, 0.5, 0.0, -0.95, 0.0])
        poses.append([0.1, -0.35, 0.65, 0.0, -0.88, 0.0])
        
        # 位姿8-10: 末端旋转 (绕Joint6旋转)
        poses.append([0.0, -0.3, 0.6, 0.0, -0.9, 0.3])
        poses.append([0.0, -0.3, 0.6, 0.0, -0.9, -0.3])
        poses.append([0.0, -0.3, 0.6, 0.0, -0.9, 0.5])
        
        # 位姿11-13: 末端倾斜 (绕Joint5旋转)
        poses.append([0.0, -0.3, 0.6, 0.0, -0.7, 0.0])
        poses.append([0.0, -0.3, 0.6, 0.0, -1.1, 0.0])
        poses.append([-0.1, -0.3, 0.6, 0.0, -0.8, 0.2])
        
        # 位姿14-16: 组合旋转
        poses.append([0.2, -0.35, 0.65, 0.1, -0.85, 0.25])
        poses.append([-0.2, -0.25, 0.55, -0.1, -0.95, -0.25])
        poses.append([0.15, -0.3, 0.6, 0.15, -0.9, 0.4])
        
        # 位姿17-20: 更多组合以增加多样性
        poses.append([-0.12, -0.32, 0.62, -0.08, -0.88, -0.2])
        poses.append([0.08, -0.28, 0.58, 0.05, -0.92, 0.15])
        poses.append([0.18, -0.38, 0.68, 0.12, -0.82, 0.35])
        poses.append([-0.18, -0.22, 0.52, -0.12, -0.98, -0.35])
        
        return poses
    
    def _image_callback(self, msg):
        """图像回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_lock:
                self.current_image = cv_image.copy()
                
                # 检测 ArUco 标记
                if self.camera_matrix is not None:
                    self._detect_aruco(cv_image)
                    
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def _camera_info_callback(self, msg):
        """相机信息回调函数"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('相机内参已接收')
            self.get_logger().info(f'相机矩阵:\n{self.camera_matrix}')
    
    def _detect_aruco(self, image):
        """检测 ArUco 标记并估计位姿"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测标记
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        self.marker_detected = False
        self.current_rvec = None
        self.current_tvec = None
        
        if ids is not None and len(ids) > 0:
            # 查找目标标记
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.marker_id:
                    # 估计位姿 (使用 OpenCV 4.7+ API)
                    obj_points = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                    )
                    
                    if success:
                        self.current_rvec = rvec.flatten()
                        self.current_tvec = tvec.flatten()
                        self.marker_detected = True
                        
                        # 添加到历史缓冲区
                        self._add_detection_to_history(self.current_rvec, self.current_tvec)
                    break
        
        # 如果没检测到，也要更新历史（添加None表示丢失帧）
        if not self.marker_detected:
            self._add_detection_to_history(None, None)
    
    def _add_detection_to_history(self, rvec, tvec):
        """添加检测结果到历史缓冲区并计算去噪结果"""
        # 添加到历史
        self.detection_rvec_history.append(rvec)
        self.detection_tvec_history.append(tvec)
        
        # 保持历史长度
        if len(self.detection_rvec_history) > self.detection_history_size:
            self.detection_rvec_history.pop(0)
            self.detection_tvec_history.pop(0)
        
        # 计算去噪结果
        self._compute_denoised_pose()
    
    def _compute_denoised_pose(self):
        """从历史数据计算去噪后的位姿"""
        # 过滤掉None值
        valid_rvecs = [r for r in self.detection_rvec_history if r is not None]
        valid_tvecs = [t for t in self.detection_tvec_history if t is not None]
        
        # 检查是否有足够的有效检测
        if len(valid_rvecs) < self.detection_min_frames:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        
        # 转换为numpy数组
        rvec_array = np.array(valid_rvecs)
        tvec_array = np.array(valid_tvecs)
        
        # 检查一致性：计算标准差，如果太大则不稳定
        tvec_std = np.std(tvec_array, axis=0)
        max_std = np.max(tvec_std)
        
        # 如果平移的标准差超过5mm，认为不稳定
        if max_std > 0.005:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        
        # 使用中值滤波去除异常值，然后取平均
        self.denoised_rvec = np.median(rvec_array, axis=0)
        self.denoised_tvec = np.median(tvec_array, axis=0)
        self.detection_stable = True
    
    def clear_detection_history(self):
        """清空检测历史"""
        self.detection_rvec_history = []
        self.detection_tvec_history = []
        self.denoised_rvec = None
        self.denoised_tvec = None
        self.detection_stable = False
    
    def _display_image(self):
        """显示图像窗口"""
        with self.image_lock:
            if self.current_image is None:
                return
            
            display_img = self.current_image.copy()
        
        # 检测并绘制 ArUco 标记
        if self.camera_matrix is not None:
            gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
                
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.marker_id:
                        # 绘制坐标轴 (使用 OpenCV 4.7+ API)
                        obj_points = np.array([
                            [-self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, -self.marker_size/2, 0],
                            [-self.marker_size/2, -self.marker_size/2, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                        )
                        
                        if success:
                            cv2.drawFrameAxes(display_img, self.camera_matrix, self.dist_coeffs,
                                             rvec, tvec, self.marker_size * 0.5)
                            
                            # 显示距离信息
                            distance = np.linalg.norm(tvec)
                            cv2.putText(display_img, f'Distance: {distance:.3f}m', 
                                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示状态信息
        status_color = (0, 255, 0) if self.marker_detected else (0, 0, 255)
        status_text = f'Marker {self.marker_id}: {"Detected" if self.marker_detected else "Not Found"}'
        cv2.putText(display_img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # 显示采样数
        samples_text = f'Samples: {len(self.R_gripper2base_samples)}/{self.min_samples}'
        cv2.putText(display_img, samples_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # 显示去噪状态
        stable_color = (0, 255, 0) if self.detection_stable else (0, 165, 255)
        stable_text = f'Stable: {"Yes" if self.detection_stable else "No"} ({len([r for r in self.detection_rvec_history if r is not None])}/{self.detection_history_size})'
        cv2.putText(display_img, stable_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, stable_color, 2)
        
        # 显示图像
        cv2.imshow('Hand-Eye Calibration - ArUco Detection', display_img)
        cv2.waitKey(1)
    
    def wait_for_services(self):
        """等待所有必要的服务"""
        self.get_logger().info('等待机械臂 Action 服务器...')
        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('无法连接到机械臂 Action 服务器')
            return False
        self.get_logger().info('机械臂 Action 服务器已连接')
        
        self.get_logger().info('等待ExecuteTrajectory服务器...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('无法连接到ExecuteTrajectory服务器')
            return False
        self.get_logger().info('ExecuteTrajectory服务器已连接')
        
        self.get_logger().info('等待运动规划服务...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('无法连接到运动规划服务')
            return False
        self.get_logger().info('运动规划服务已连接')
        
        self.get_logger().info('等待相机数据...')
        timeout = 30.0
        start_time = time.time()
        while not self.camera_info_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('等待相机数据超时')
                return False
        
        self.get_logger().info('相机数据已就绪')
        return True
    
    def plan_to_joint_positions(self, joint_positions):
        """
        使用MoveIt规划到目标关节位置的轨迹
        
        Args:
            joint_positions: 目标关节位置列表
            
        Returns:
            规划好的轨迹或None（规划失败时）
        """
        request = GetMotionPlan.Request()
        
        # 设置规划组
        request.motion_plan_request.group_name = "Alicia"
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        
        # 使用当前状态作为起始状态
        request.motion_plan_request.start_state.is_diff = True
        
        # 设置关节目标约束
        goal_constraints = Constraints()
        goal_constraints.name = "joint_goal"
        
        for i, joint_name in enumerate(self.arm_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_positions[i])
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        request.motion_plan_request.goal_constraints.append(goal_constraints)
        
        # 调用规划服务
        try:
            self.get_logger().info(f'规划到关节位置: {[f"{j:.3f}" for j in joint_positions]}')
            future = self.plan_client.call_async(request)
            
            # 非阻塞等待，保持图像更新
            timeout_sec = 10.0
            start_time = time.time()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('运动规划服务超时')
                    return None
            
            response = future.result()
            
            if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f'规划成功，轨迹包含 {len(response.motion_plan_response.trajectory.joint_trajectory.points)} 个点')
                return response.motion_plan_response.trajectory
            else:
                self.get_logger().error(f'运动规划失败，错误码: {response.motion_plan_response.error_code.val}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'运动规划服务调用失败: {e}')
            return None
    
    def execute_trajectory(self, trajectory):
        """
        执行规划好的轨迹（非阻塞方式，保持图像更新）
        
        Args:
            trajectory: RobotTrajectory消息
            
        Returns:
            bool: 是否执行成功
        """
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('执行轨迹...')
        
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        
        # 非阻塞等待目标接受
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)  # 保持回调处理
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待轨迹执行目标接受超时')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('轨迹执行目标被拒绝')
            return False
        
        self.get_logger().info('轨迹执行目标已接受，等待完成...')
        
        # 根据轨迹时长计算超时
        if trajectory.joint_trajectory.points:
            last_point = trajectory.joint_trajectory.points[-1]
            traj_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            timeout_sec = traj_duration + 15.0
        else:
            timeout_sec = 30.0
        
        result_future = goal_handle.get_result_async()
        
        # 非阻塞等待结果，同时保持图像更新
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)  # 保持回调处理
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待轨迹执行结果超时')
                return False
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('轨迹执行完成')
            return True
        else:
            self.get_logger().error(f'轨迹执行失败，错误码: {result.result.error_code.val}')
            return False
    
    def move_to_pose(self, joint_positions, duration_sec=4.0):
        """
        移动机械臂到指定关节位置（使用MoveIt plan+execute）
        
        Args:
            joint_positions: 关节位置列表 (弧度)
            duration_sec: 运动时长 (秒) - 仅作为备用方法的参数
        
        Returns:
            bool: 是否成功
        """
        self.get_logger().info(f'移动到位姿: {[f"{p:.3f}" for p in joint_positions]} (使用MoveIt)')
        
        # 使用MoveIt plan+execute
        trajectory = self.plan_to_joint_positions(joint_positions)
        if trajectory is not None:
            if self.execute_trajectory(trajectory):
                time.sleep(0.5)  # 等待机械臂稳定
                return True
        
        # 如果MoveIt失败，回退到直接轨迹控制
        self.get_logger().warn('MoveIt plan+execute失败，尝试直接轨迹控制...')
        return self._move_to_pose_direct(joint_positions, duration_sec)
    
    def _move_to_pose_direct(self, joint_positions, duration_sec=4.0):
        """
        直接使用FollowJointTrajectory控制机械臂（备用方法）
        
        Args:
            joint_positions: 关节位置列表 (弧度)
            duration_sec: 运动时长 (秒)
        
        Returns:
            bool: 是否成功
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'直接控制移动到位姿: {[f"{p:.3f}" for p in joint_positions]}')
        
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        
        # 非阻塞等待
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待目标接受超时')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        result_future = goal_handle.get_result_async()
        
        # 非阻塞等待结果
        timeout_sec = duration_sec + 10.0
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待运动结果超时')
                return False
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('到达目标位置')
            return True
        else:
            self.get_logger().error(f'运动失败，错误码: {result.result.error_code}')
            return False
    
    def get_end_effector_pose(self):
        """
        获取末端执行器相对于基座的位姿
        
        Returns:
            tuple: (rotation_matrix, translation_vector) 或 None
        """
        try:
            # 等待变换可用
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # 提取旋转和平移
            q = transform.transform.rotation
            t = transform.transform.translation
            
            # 四元数转旋转矩阵
            rotation = R.from_quat([q.x, q.y, q.z, q.w])
            rotation_matrix = rotation.as_matrix()
            
            translation_vector = np.array([t.x, t.y, t.z])
            
            return rotation_matrix, translation_vector
            
        except Exception as e:
            self.get_logger().error(f'获取末端位姿失败: {e}')
            return None
    
    def get_marker_pose(self):
        """
        获取 ArUco 标记相对于相机的位姿（使用去噪后的结果）
        
        Returns:
            tuple: (rotation_matrix, translation_vector) 或 None
        """
        # 优先使用去噪后的稳定结果
        if self.detection_stable and self.denoised_rvec is not None:
            rotation_matrix, _ = cv2.Rodrigues(self.denoised_rvec)
            translation_vector = self.denoised_tvec
            return rotation_matrix, translation_vector
        
        # 回退到单帧结果
        if not self.marker_detected or self.current_rvec is None:
            return None
        
        # 旋转向量转旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues(self.current_rvec)
        translation_vector = self.current_tvec
        
        return rotation_matrix, translation_vector
    
    def record_sample(self):
        """
        记录当前采样点
        
        Returns:
            bool: 是否成功记录
        """
        # 获取末端位姿
        ee_pose = self.get_end_effector_pose()
        if ee_pose is None:
            self.get_logger().warn('无法获取末端位姿')
            return False
        
        # 获取标记位姿
        marker_pose = self.get_marker_pose()
        if marker_pose is None:
            self.get_logger().warn('无法获取标记位姿 (标记未检测到)')
            return False
        
        R_gripper2base, t_gripper2base = ee_pose
        R_target2cam, t_target2cam = marker_pose
        
        # 存储采样数据
        self.R_gripper2base_samples.append(R_gripper2base)
        self.t_gripper2base_samples.append(t_gripper2base)
        self.R_target2cam_samples.append(R_target2cam)
        self.t_target2cam_samples.append(t_target2cam)
        
        sample_count = len(self.R_gripper2base_samples)
        self.get_logger().info(f'成功记录采样点 #{sample_count}')
        self.get_logger().info(f'  末端位置: [{t_gripper2base[0]:.4f}, {t_gripper2base[1]:.4f}, {t_gripper2base[2]:.4f}]')
        self.get_logger().info(f'  标记距离: {np.linalg.norm(t_target2cam):.4f} m')
        
        return True
    
    def compute_calibration(self):
        """
        计算手眼标定结果
        
        Returns:
            tuple: (rotation_matrix, translation_vector) 相机到末端的变换
        """
        if len(self.R_gripper2base_samples) < 3:
            self.get_logger().error('采样数量不足，至少需要3个采样点')
            return None
        
        self.get_logger().info(f'开始计算手眼标定 (共 {len(self.R_gripper2base_samples)} 个采样点)')
        
        # 选择标定算法
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF
        }
        
        if self.calibration_method not in method_map:
            self.get_logger().warn(f'未知的标定算法: {self.calibration_method}, 使用 tsai')
            self.calibration_method = 'tsai'
        
        method = method_map[self.calibration_method]
        
        # 转换为 OpenCV 格式
        R_gripper2base = [r for r in self.R_gripper2base_samples]
        t_gripper2base = [t.reshape(3, 1) for t in self.t_gripper2base_samples]
        R_target2cam = [r for r in self.R_target2cam_samples]
        t_target2cam = [t.reshape(3, 1) for t in self.t_target2cam_samples]
        
        # 执行手眼标定
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method
            )
            
            self.get_logger().info('手眼标定计算完成')
            return R_cam2gripper, t_cam2gripper.flatten()
            
        except Exception as e:
            self.get_logger().error(f'手眼标定计算失败: {e}')
            return None
    
    def compute_with_multiple_methods(self):
        """
        使用多种算法计算并比较结果
        
        Returns:
            dict: 各算法的结果
        """
        methods = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF
        }
        
        results = {}
        
        R_gripper2base = [r for r in self.R_gripper2base_samples]
        t_gripper2base = [t.reshape(3, 1) for t in self.t_gripper2base_samples]
        R_target2cam = [r for r in self.R_target2cam_samples]
        t_target2cam = [t.reshape(3, 1) for t in self.t_target2cam_samples]
        
        for name, method in methods.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method
                )
                results[name] = {
                    'R': R,
                    't': t.flatten(),
                    'success': True
                }
            except Exception as e:
                results[name] = {
                    'error': str(e),
                    'success': False
                }
        
        return results
    
    def save_result(self, R_cam2gripper, t_cam2gripper):
        """
        保存标定结果到 YAML 文件
        
        Args:
            R_cam2gripper: 旋转矩阵 (相机到末端)
            t_cam2gripper: 平移向量 (相机到末端)
        """
        # 计算四元数
        rotation = R.from_matrix(R_cam2gripper)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        euler = rotation.as_euler('xyz', degrees=True)
        
        # 构建 4x4 变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R_cam2gripper
        transform_matrix[:3, 3] = t_cam2gripper
        
        result = {
            'hand_eye_calibration': {
                'type': 'eye_in_hand',
                'frame_id': self.end_effector_link,
                'child_frame_id': 'camera_link',
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'samples_count': len(self.R_gripper2base_samples),
                'calibration_method': self.calibration_method,
                'aruco_marker_id': self.marker_id,
                'aruco_marker_size': self.marker_size,
                'transform': {
                    'translation': {
                        'x': float(t_cam2gripper[0]),
                        'y': float(t_cam2gripper[1]),
                        'z': float(t_cam2gripper[2])
                    },
                    'rotation': {
                        'quaternion': {
                            'x': float(quaternion[0]),
                            'y': float(quaternion[1]),
                            'z': float(quaternion[2]),
                            'w': float(quaternion[3])
                        },
                        'euler_xyz_deg': {
                            'roll': float(euler[0]),
                            'pitch': float(euler[1]),
                            'yaw': float(euler[2])
                        }
                    },
                    'matrix_4x4': transform_matrix.tolist()
                }
            }
        }
        
        # 使用 ROS2 包路径查找源码目录
        try:
            # 获取包的 share 目录（install 目录）
            package_share_dir = get_package_share_directory('alicia_d_calibration')
            # 从 install 目录推断源码目录
            # install/alicia_d_calibration/share/alicia_d_calibration -> src/alicia_d_calibration
            workspace_root = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..'))
            config_dir = os.path.join(workspace_root, 'src', 'alicia_d_calibration', 'config')
        except Exception as e:
            # 如果获取包路径失败，回退到使用脚本相对路径
            self.get_logger().warn(f'无法获取包路径: {e}，使用脚本相对路径')
            script_dir = os.path.dirname(__file__)
            config_dir = os.path.join(script_dir, '..', 'config')
        
        output_path = os.path.join(config_dir, self.output_file)
        
        # 确保目录存在
        os.makedirs(config_dir, exist_ok=True)
        
        with open(output_path, 'w') as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True)
        
        # 显示绝对路径以便用户确认位置
        abs_path = os.path.abspath(output_path)
        self.get_logger().info(f'标定结果已保存到: {abs_path}')
        
        # 打印结果
        self.get_logger().info('=' * 60)
        self.get_logger().info('手眼标定结果 (相机到末端执行器)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'平移 (m): x={t_cam2gripper[0]:.6f}, y={t_cam2gripper[1]:.6f}, z={t_cam2gripper[2]:.6f}')
        self.get_logger().info(f'旋转 (四元数): x={quaternion[0]:.6f}, y={quaternion[1]:.6f}, z={quaternion[2]:.6f}, w={quaternion[3]:.6f}')
        self.get_logger().info(f'旋转 (欧拉角, 度): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}')
        self.get_logger().info('=' * 60)
        
        return output_path
    
    def run_calibration(self):
        """运行标定流程"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始手眼标定流程')
        self.get_logger().info('=' * 60)
        
        # 等待服务就绪
        if not self.wait_for_services():
            return False
        
        # 先回到安全位姿
        home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('移动到初始位置...')
        if not self.move_to_pose(home_pose, 3.0):
            self.get_logger().error('无法移动到初始位置')
            return False
        
        time.sleep(1.0)
        
        # 遍历标定位姿
        pose_idx = 0
        while len(self.R_gripper2base_samples) < self.min_samples and pose_idx < len(self.calibration_poses):
            self.get_logger().info(f'\n--- 位姿 {pose_idx + 1}/{len(self.calibration_poses)} ---')
            
            pose = self.calibration_poses[pose_idx]
            
            # 移动到目标位姿
            if not self.move_to_pose(pose, 4.0):
                self.get_logger().warn(f'位姿 {pose_idx + 1} 移动失败，跳过')
                pose_idx += 1
                continue
            
            # 等待机械臂物理稳定
            self.get_logger().info('等待机械臂稳定 ...')
            settle_start = time.time()
            while time.time() - settle_start < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)  # 保持图像更新
            
            # 清空检测历史，准备收集新的稳定检测
            self.clear_detection_history()
            
            # 收集足够的检测数据并等待稳定
            self.get_logger().info('收集检测数据...')
            stable_wait_start = time.time()
            max_stable_wait = 5.0  # 最多等待5秒
            
            while time.time() - stable_wait_start < max_stable_wait:
                rclpy.spin_once(self, timeout_sec=0.05)  # 保持回调处理
                
                # 检查是否已经稳定
                if self.detection_stable:
                    self.get_logger().info('检测已稳定')
                    break
            
            # 检查标记是否可见且稳定
            if not self.detection_stable:
                self.get_logger().warn(f'位姿 {pose_idx + 1}: 标记检测不稳定，跳过')
                pose_idx += 1
                continue
            
            # 标记检测稳定，自动记录采样点
            self.get_logger().info(f'位姿 {pose_idx + 1}: 标记检测稳定，自动记录采样点...')
            if self.record_sample():
                self.get_logger().info(f'✓ 已记录采样点 {len(self.R_gripper2base_samples)}/{self.min_samples}')
            else:
                self.get_logger().warn(f'✗ 采样点记录失败，跳过')
            
            pose_idx += 1
        
        # 回到初始位置
        self.get_logger().info('返回初始位置...')
        self.move_to_pose(home_pose, 3.0)
        
        # 检查采样数量
        if len(self.R_gripper2base_samples) < 3:
            self.get_logger().error(f'采样数量不足: {len(self.R_gripper2base_samples)} (最少需要3个)')
            return False
        
        if len(self.R_gripper2base_samples) < self.min_samples:
            self.get_logger().warn(f'采样数量低于推荐值: {len(self.R_gripper2base_samples)}/{self.min_samples}，但继续计算')
        
        # 使用多种算法计算并比较
        self.get_logger().info('\n使用多种算法计算手眼标定...')
        all_results = self.compute_with_multiple_methods()
        
        # 打印所有算法结果
        self.get_logger().info('\n各算法计算结果比较:')
        for name, result in all_results.items():
            if result['success']:
                t = result['t']
                self.get_logger().info(f'  {name}: t=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]')
            else:
                self.get_logger().warn(f'  {name}: 计算失败 - {result.get("error", "未知错误")}')
        
        # 使用选定的算法结果
        if self.calibration_method in all_results and all_results[self.calibration_method]['success']:
            R_result = all_results[self.calibration_method]['R']
            t_result = all_results[self.calibration_method]['t']
        else:
            # 使用第一个成功的结果
            for name, result in all_results.items():
                if result['success']:
                    R_result = result['R']
                    t_result = result['t']
                    self.calibration_method = name
                    break
            else:
                self.get_logger().error('所有算法都计算失败')
                return False
        
        # 保存结果
        self.save_result(R_result, t_result)
        
        self.get_logger().info('手眼标定完成!')
        return True
    
    def shutdown(self):
        """关闭节点"""
        cv2.destroyAllWindows()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = HandEyeCalibration()
    
    try:
        success = node.run_calibration()
        if success:
            print('\n标定完成! 请查看保存的标定结果文件。')
        else:
            print('\n标定失败或被用户取消。')
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
