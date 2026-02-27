#!/usr/bin/env python3
"""
Grasp Execution (Intel RealSense D405)
Environment: System Python (outside conda, with ROS 2)

Subscribes to grasp poses from GraspGen, transforms them to robot base frame,
filters invalid poses, solves IK, and executes grasps using MoveIt 2.

This script handles:
1. Coordinate transformation from camera frame to robot base frame (using TF2)
2. Gripper convention alignment (GraspGen X=open -> actual Y=open)
3. Gripper offset correction (base center to fingertip)
4. Pose filtering (z height, approach direction)
5. IK solving with MoveIt 2
6. Grasp execution with user confirmation

IMPORTANT: The coordinate transformation uses TF2 to get the full transform chain
from camera_depth_optical_frame to base_link. This properly handles the optical
frame conventions used by depth cameras.

Usage:
    # Outside conda environment (system Python with ROS 2)
    python d405_execution.py [options]
"""

import os
import sys
import argparse
import time
import threading
import yaml
import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Header, Float32MultiArray, Int32
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import (
    RobotState, Constraints, JointConstraint,
    PositionIKRequest, MoveItErrorCodes, MotionPlanRequest
)

from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R

# Configuration paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_PATH = os.path.join(
    SCRIPT_DIR, '..', '..', '..', 'alicia_d_calibration', 'config', 'hand_eye_calibration_result.yaml'
)


class GraspExecutionNode(Node):
    """
    ROS 2 Node for grasp execution using MoveIt 2 (Intel RealSense D405).
    """
    
    # Default HOME position (joint angles in radians)
    HOME_POSITION = [0.0, 0.8722, -0.0174, 0.0, -1.1164, 0.0]
    
    # Gripper configuration
    GRIPPER_DEPTH = 0.12  # Distance from gripper base center to fingertip (m)
    GRIPPER_OPEN = [0.0]  # Gripper open position
    GRIPPER_CLOSED = [0.024]  # Gripper closed position
    
    # Movement parameters
    PRE_GRASP_DISTANCE = 0.08  # Pre-grasp approach distance (m)
    LIFT_HEIGHT = 0.0  # Lift height after grasp (m)
    
    # Frame configuration - must match what grasp_generation.py uses
    # Point cloud and grasp poses are in the depth camera's optical frame
    CAMERA_FRAME = 'camera_depth_optical_frame'
    BASE_FRAME = 'base_link'
    
    # Coordinate offset correction (in base frame, meters)
    # Use these to fine-tune the grasp position if there's systematic error
    OFFSET_X = -0.0   # X offset in base frame
    OFFSET_Y = -0.0   # Y offset in base frame
    OFFSET_Z = 0.0    # Z offset in base frame
    
    def __init__(self, args):
        super().__init__('grasp_execution_node_d405')
        
        self.args = args
        
        # Apply offset corrections from command line arguments
        self.OFFSET_X = args.offset_x
        self.OFFSET_Y = args.offset_y
        self.OFFSET_Z = args.offset_z
        
        self.get_logger().info(f"Offset corrections: X={self.OFFSET_X:.3f}m, Y={self.OFFSET_Y:.3f}m, Z={self.OFFSET_Z:.3f}m")
        
        # Callback groups for async operations
        self.action_callback_group = ReentrantCallbackGroup()
        
        # Joint names
        self.arm_joint_names = [
            'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'
        ]
        self.gripper_joint_names = ['Gripper']
        
        # Current state
        self.current_joint_state = None
        self.joint_state_lock = threading.Lock()
        
        # Grasp data
        self.grasp_poses = []  # List of 4x4 matrices
        self.grasp_confidences = []
        self.grasp_lock = threading.Lock()
        self.new_grasps_available = False
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = False
        
        # Static TF broadcaster for hand-eye calibration
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish hand-eye calibration as static TF
        self._publish_hand_eye_tf()
        
        # Initialize ROS interfaces
        self._init_ros_interfaces()
        
        # Workflow state machine
        self.workflow_state = "init"
        self.selected_grasp = None
        self.ik_solutions = []
        self.current_grasp_index = 0
        
        # Timer for workflow execution
        self.workflow_timer = self.create_timer(0.5, self._workflow_tick)
        
        self.get_logger().info("Grasp Execution Node (D405) initialized")
        self.get_logger().info("Workflow will start automatically...")
    
    def _publish_hand_eye_tf(self):
        """
        Publish hand-eye calibration as static TF.
        
        This connects gripper_center -> camera_link, enabling the full TF chain:
        camera_depth_optical_frame -> camera_link -> gripper_center -> ... -> base_link
        """
        if not os.path.exists(CALIBRATION_PATH):
            self.get_logger().error(f"Calibration file not found: {CALIBRATION_PATH}")
            self.get_logger().error("TF will not be published. Run the camera driver and ensure")
            self.get_logger().error("hand-eye calibration TF is published by another node.")
            return
        
        try:
            with open(CALIBRATION_PATH, 'r') as f:
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
            
            # Transform from optical frame to link frame (D405)
            # The D405 camera_link to camera_depth_optical_frame is a standard optical transform
            m_internal = np.array([
                [ 0.0, -0.0,  1.0, 0.0],
                [-1.0, -0.0, -0.0, 0.0],
                [ 0.0, -1.0,  0.0, 0.0],
                [ 0.0,  0.0,  0.0, 1.0]
            ])
            
            # Final transform: T_gripper_link = T_gripper_optical * inv(T_link_optical)
            m_final = m_calib @ np.linalg.inv(m_internal)
            
            final_t = m_final[:3, 3]
            final_q = R.from_matrix(m_final[:3, :3]).as_quat()
            
            parent_frame = hand_eye.get('frame_id', 'gripper_center')
            child_frame = 'camera_link'
            
            # Create and publish static transform
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = parent_frame
            static_tf.child_frame_id = child_frame
            static_tf.transform.translation.x = float(final_t[0])
            static_tf.transform.translation.y = float(final_t[1])
            static_tf.transform.translation.z = float(final_t[2])
            static_tf.transform.rotation.x = float(final_q[0])
            static_tf.transform.rotation.y = float(final_q[1])
            static_tf.transform.rotation.z = float(final_q[2])
            static_tf.transform.rotation.w = float(final_q[3])
            
            self.static_broadcaster.sendTransform(static_tf)
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("Published hand-eye calibration TF:")
            self.get_logger().info(f"  {parent_frame} -> {child_frame}")
            self.get_logger().info(f"  Translation: [{final_t[0]:.4f}, {final_t[1]:.4f}, {final_t[2]:.4f}]")
            euler = R.from_quat(final_q).as_euler('xyz', degrees=True)
            self.get_logger().info(f"  Rotation (deg): [{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}]")
            self.get_logger().info("=" * 60)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish hand-eye TF: {e}")
    
    def _check_tf_available(self) -> bool:
        """Check if TF transform is available between camera and base frames."""
        try:
            self.tf_buffer.lookup_transform(
                self.BASE_FRAME, self.CAMERA_FRAME,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            if not self.tf_ready:
                self.tf_ready = True
                self.get_logger().info(f'TF transform ready: {self.CAMERA_FRAME} -> {self.BASE_FRAME}')
            return True
        except Exception as e:
            if self.tf_ready:
                self.get_logger().warn(f'TF transform became unavailable: {e}')
                self.tf_ready = False
            return False
    
    def _init_ros_interfaces(self):
        """Initialize ROS subscribers, publishers, and action clients."""
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, sensor_qos)
        
        # Grasp poses subscriber
        self.grasp_sub = self.create_subscription(
            PoseArray, '/grasp_6d/grasp_poses',
            self._grasp_poses_callback, reliable_qos)
        
        # Grasp confidences subscriber
        self.conf_sub = self.create_subscription(
            Float32MultiArray, '/grasp_6d/grasp_confidences',
            self._grasp_confidences_callback, reliable_qos)
        
        # 高亮索引发布者
        self.highlight_pub = self.create_publisher(
            Int32, '/grasp_6d/highlight_index', reliable_qos)
        
        # Action clients
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/Alicia_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group)
        
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/Gripper_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group)
        
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.action_callback_group)
        
        # Wait for action servers
        self.get_logger().info('Waiting for arm action server...')
        self.arm_action_client.wait_for_server()
        self.get_logger().info('Arm action server connected!')
        
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Gripper action server connected!')
        
        self.get_logger().info('Waiting for ExecuteTrajectory action server...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('ExecuteTrajectory action server connected!')
        
        # Service clients
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service()
        self.get_logger().info('IK service connected!')
        
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for motion planning service...')
        self.plan_client.wait_for_service()
        self.get_logger().info('Motion planning service connected!')
        
        self.get_logger().info("ROS interfaces initialized (D405)")
    
    def _joint_state_callback(self, msg: JointState):
        """Store current joint state."""
        with self.joint_state_lock:
            self.current_joint_state = msg
    
    def _grasp_poses_callback(self, msg: PoseArray):
        """Process incoming grasp poses."""
        poses = []
        for pose in msg.poses:
            # Convert Pose to 4x4 matrix
            T = np.eye(4)
            T[0, 3] = pose.position.x
            T[1, 3] = pose.position.y
            T[2, 3] = pose.position.z
            
            q = [pose.orientation.x, pose.orientation.y, 
                 pose.orientation.z, pose.orientation.w]
            T[:3, :3] = R.from_quat(q).as_matrix()
            
            poses.append(T)
        
        with self.grasp_lock:
            self.grasp_poses = poses
            self.new_grasps_available = True
        
        self.get_logger().info(f"Received {len(poses)} grasp poses")
    
    def _grasp_confidences_callback(self, msg: Float32MultiArray):
        """Store grasp confidences."""
        with self.grasp_lock:
            self.grasp_confidences = list(msg.data)
    
    def transform_grasp_to_base(self, grasp_cam: np.ndarray) -> np.ndarray:
        """Transform grasp pose from camera optical frame to robot base frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.BASE_FRAME, self.CAMERA_FRAME,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            if not self.tf_ready:
                self.tf_ready = True
                self.get_logger().info(f'TF transform ready: {self.CAMERA_FRAME} -> {self.BASE_FRAME}')
            
            t = transform.transform.translation
            r = transform.transform.rotation
            
            T_base_to_cam = np.eye(4)
            T_base_to_cam[:3, :3] = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            T_base_to_cam[:3, 3] = [t.x, t.y, t.z]
            
            grasp_base = T_base_to_cam @ grasp_cam
            
            # Gripper convention alignment
            R_align = R.from_euler('z', -90, degrees=True).as_matrix()
            grasp_base[:3, :3] = grasp_base[:3, :3] @ R_align
            
            # Gripper offset correction
            if self.GRIPPER_DEPTH > 0:
                approach_dir = grasp_base[:3, 2]
                offset = approach_dir * self.GRIPPER_DEPTH
                grasp_base[:3, 3] += offset
            
            # Apply offset corrections
            grasp_base[0, 3] += self.OFFSET_X
            grasp_base[1, 3] += self.OFFSET_Y
            grasp_base[2, 3] += self.OFFSET_Z
            
            return grasp_base
            
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return None
    
    def filter_grasp_poses(self, grasps: List[np.ndarray], 
                          confidences: List[float]) -> List[Tuple[int, np.ndarray, float]]:
        """Filter grasp poses based on validity criteria."""
        valid_grasps = []
        
        for i, (grasp, conf) in enumerate(zip(grasps, confidences)):
            z = grasp[2, 3]
            approach_z = grasp[2, 2]
            
            if z < self.args.min_z:
                continue
            
            if approach_z > 0:
                continue
            
            valid_grasps.append((i, grasp, conf))
        
        valid_grasps.sort(key=lambda x: x[2], reverse=True)
        
        self.get_logger().info(f"Filtered grasps: {len(valid_grasps)}/{len(grasps)} valid")
        
        return valid_grasps
    
    def compute_ik(self, position: np.ndarray, orientation: np.ndarray, 
                   timeout_sec: float = 1.0) -> Optional[List[float]]:
        """Compute inverse kinematics for a target pose."""
        if not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("IK service not available")
            return None
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = "Alicia"
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int((timeout_sec % 1) * 1e9)
        request.ik_request.ik_link_name = "gripper_center"
        
        with self.joint_state_lock:
            if self.current_joint_state is not None:
                request.ik_request.robot_state.joint_state = self.current_joint_state
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(position[0])
        pose_stamped.pose.position.y = float(position[1])
        pose_stamped.pose.position.z = float(position[2])
        pose_stamped.pose.orientation.x = float(orientation[0])
        pose_stamped.pose.orientation.y = float(orientation[1])
        pose_stamped.pose.orientation.z = float(orientation[2])
        pose_stamped.pose.orientation.w = float(orientation[3])
        request.ik_request.pose_stamped = pose_stamped
        
        future = self.ik_client.call_async(request)
        start_time = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_time > timeout_sec + 0.5:
                return None
        
        response = future.result()
        
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        
        joint_positions = []
        for joint_name in self.arm_joint_names:
            if joint_name in response.solution.joint_state.name:
                idx = response.solution.joint_state.name.index(joint_name)
                joint_positions.append(response.solution.joint_state.position[idx])
        
        if len(joint_positions) != len(self.arm_joint_names):
            return None
        
        return joint_positions
    
    def compute_joint_distance(self, target_joints: List[float]) -> float:
        """Compute joint angle distance from current to target."""
        with self.joint_state_lock:
            if self.current_joint_state is None:
                return float('inf')
            
            current = []
            for name in self.arm_joint_names:
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    current.append(self.current_joint_state.position[idx])
            
            if len(current) != len(target_joints):
                return float('inf')
        
        return np.sum(np.abs(np.array(current) - np.array(target_joints)))
    
    def plan_to_joint_positions(self, joint_positions: List[float]):
        """Plan a trajectory to the specified joint positions."""
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Motion planning service not available")
            return None
        
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = "Alicia"
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        request.motion_plan_request.start_state.is_diff = True
        
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
        
        try:
            future = self.plan_client.call_async(request)
            
            timeout_sec = 10.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error("Motion planning timeout")
                    return None
                time.sleep(0.05)
            
            response = future.result()
            
            if response.motion_plan_response.error_code.val == 1:
                return response.motion_plan_response.trajectory
            else:
                self.get_logger().error(f"Motion planning failed: {response.motion_plan_response.error_code.val}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Motion planning service call failed: {e}")
            return None
    
    def execute_trajectory(self, trajectory) -> bool:
        """Execute a planned trajectory."""
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available")
            return False
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for trajectory goal acceptance")
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory execution goal rejected")
            return False
        
        if trajectory.joint_trajectory.points:
            last_point = trajectory.joint_trajectory.points[-1]
            traj_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            timeout_sec = traj_duration + 15.0
        else:
            timeout_sec = 30.0
        
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout waiting for trajectory execution")
                return False
        
        result = result_future.result()
        if result.result.error_code.val == 1:
            return True
        else:
            self.get_logger().error(f"Trajectory execution failed: {result.result.error_code.val}")
            return False
    
    def move_arm_with_moveit(self, joint_positions: List[float]) -> bool:
        """Move arm using MoveIt plan + execute."""
        trajectory = self.plan_to_joint_positions(joint_positions)
        if trajectory is None:
            self.get_logger().error("Failed to plan trajectory")
            return False
        
        success = self.execute_trajectory(trajectory)
        if success:
            time.sleep(0.5)
        return success
    
    def _move_arm_direct(self, joint_positions: List[float], duration_sec: float = 3.0) -> bool:
        """Move arm using direct FollowJointTrajectory action."""
        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm action server not available")
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(sec=int(duration_sec), 
                                         nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points.append(point)
        
        future = self.arm_action_client.send_goal_async(goal)
        
        timeout_sec = 5.0
        start_time = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Timeout sending arm goal")
                return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        timeout_sec = duration_sec + 10.0
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error("Arm movement timed out")
                return False
        
        result = result_future.result()
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Arm movement failed: {result.result.error_code}")
            return False
        
        return True
    
    def move_arm_to_joints(self, joint_positions: List[float], 
                          duration_sec: float = 3.0) -> bool:
        """Move arm to target joint positions using MoveIt plan+execute."""
        self.get_logger().info(f"Moving arm to: {[f'{j:.3f}' for j in joint_positions]}")
        
        success = self.move_arm_with_moveit(joint_positions)
        
        if success:
            self.get_logger().info("Arm movement completed")
            return True
        
        self.get_logger().warning("MoveIt failed, trying direct trajectory control...")
        success = self._move_arm_direct(joint_positions, duration_sec)
        
        if success:
            time.sleep(0.5)
            self.get_logger().info("Arm movement completed (direct)")
            return True
        
        return False
    
    def move_gripper(self, position: float, duration_sec: float = 1.5) -> bool:
        """Move gripper to target position."""
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available")
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec),
                                         nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points.append(point)
        
        action = "closing" if position > 0.01 else "opening"
        self.get_logger().info(f"Gripper {action}...")
        
        send_goal_future = self.gripper_action_client.send_goal_async(goal)
        
        timeout_sec_goal = 5.0
        start_time = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec_goal:
                self.get_logger().error("Timeout sending gripper goal")
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        timeout_result = duration_sec + 5.0
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_result:
                self.get_logger().error("Gripper movement timed out")
                return False
        
        return True
    
    def open_gripper(self) -> bool:
        """Open the gripper."""
        return self.move_gripper(self.GRIPPER_OPEN[0])
    
    def close_gripper(self) -> bool:
        """Close the gripper."""
        return self.move_gripper(self.GRIPPER_CLOSED[0])
    
    def move_to_home(self) -> bool:
        """Move arm to HOME position."""
        self.get_logger().info("Moving to HOME position...")
        return self.move_arm_to_joints(self.HOME_POSITION, duration_sec=3.0)
    
    def move_to_pose(self, pose: np.ndarray, duration_sec: float = 2.0) -> bool:
        """Move arm to a Cartesian pose using IK."""
        position = pose[:3, 3]
        orientation = R.from_matrix(pose[:3, :3]).as_quat()
        
        joint_positions = self.compute_ik(position, orientation)
        if joint_positions is None:
            return False
        
        return self.move_arm_to_joints(joint_positions, duration_sec)
    
    def compute_pre_grasp_pose(self, grasp_pose: np.ndarray) -> np.ndarray:
        """Compute pre-grasp pose (retracted along approach direction)."""
        pre_grasp = grasp_pose.copy()
        approach_dir = grasp_pose[:3, 2]
        pre_grasp[:3, 3] -= approach_dir * self.PRE_GRASP_DISTANCE
        return pre_grasp
    
    def compute_lift_pose(self, grasp_pose: np.ndarray) -> np.ndarray:
        """Compute lift pose (raised in Z direction)."""
        lift_pose = grasp_pose.copy()
        lift_pose[2, 3] += self.LIFT_HEIGHT
        return lift_pose
    
    def highlight_grasp_in_meshcat(self, orig_idx: int):
        """发布高亮索引，让 grasp_generation.py 在 meshcat 中高亮对应的夹爪。"""
        msg = Int32()
        msg.data = orig_idx
        self.highlight_pub.publish(msg)
        self.get_logger().debug(f"Published highlight index: {orig_idx}")
    
    def clear_highlight_in_meshcat(self):
        """发布 -1 清除高亮。"""
        msg = Int32()
        msg.data = -1
        self.highlight_pub.publish(msg)
    
    def execute_grasp(self, grasp_pose: np.ndarray) -> bool:
        """Execute a grasp at the given pose."""
        self.get_logger().info("Executing grasp sequence...")
        
        pre_grasp = self.compute_pre_grasp_pose(grasp_pose)
        self.get_logger().info("Step 1: Moving to pre-grasp position...")
        if not self.move_to_pose(pre_grasp, duration_sec=2.0):
            self.get_logger().error("Failed to reach pre-grasp position")
            return False
        time.sleep(0.5)
        
        self.get_logger().info("Step 2: Opening gripper...")
        self.open_gripper()
        time.sleep(0.5)
        
        self.get_logger().info("Step 3: Moving to grasp position...")
        if not self.move_to_pose(grasp_pose, duration_sec=1.5):
            self.get_logger().error("Failed to reach grasp position")
            return False
        time.sleep(0.5)
        
        self.get_logger().info("Step 4: Closing gripper...")
        self.close_gripper()
        time.sleep(1.0)
        
        lift_pose = self.compute_lift_pose(grasp_pose)
        self.get_logger().info("Step 5: Lifting object...")
        if not self.move_to_pose(lift_pose, duration_sec=1.5):
            self.get_logger().error("Failed to lift object")
            return False
        
        self.get_logger().info("Grasp sequence completed!")
        return True
    
    def _workflow_tick(self):
        """Timer-based workflow execution."""
        if self.workflow_state == "init":
            self.get_logger().info("=" * 60)
            self.get_logger().info("Grasp Execution Node Running (D405)")
            self.get_logger().info("=" * 60)
            
            # Open gripper first
            self.get_logger().info("Opening gripper...")
            self.open_gripper()
            time.sleep(0.5)
            
            self.get_logger().info("Moving to HOME position...")
            
            if self.move_to_home():
                self.get_logger().info("Waiting for grasp poses from /grasp_6d/grasp_poses...")
                self.workflow_state = "wait_grasps"
            else:
                self.get_logger().error("Failed to move to HOME position, retrying...")
        
        elif self.workflow_state == "wait_grasps":
            with self.grasp_lock:
                if not self.new_grasps_available:
                    return
                
                grasp_poses_cam = self.grasp_poses.copy()
                confidences = self.grasp_confidences.copy()
                self.new_grasps_available = False
            
            if len(grasp_poses_cam) == 0:
                return
            
            self.get_logger().info(f"\nProcessing {len(grasp_poses_cam)} grasp poses...")
            
            if not self._check_tf_available():
                self.get_logger().warn("TF not available, waiting...")
                return
            
            grasps_base = []
            for grasp_cam in grasp_poses_cam:
                grasp_base = self.transform_grasp_to_base(grasp_cam)
                if grasp_base is not None:
                    grasps_base.append(grasp_base)
            
            if len(grasps_base) == 0:
                self.get_logger().warn("No grasps could be transformed to base frame!")
                return
            
            valid_grasps = self.filter_grasp_poses(grasps_base, confidences)
            
            if len(valid_grasps) == 0:
                self.get_logger().warn("No valid grasps after filtering!")
                return
            
            def compute_presort_score(item):
                orig_idx, grasp, conf = item
                approach_dir = grasp[:3, 2]
                topdown_score = -approach_dir[2]
                pos = grasp[:3, 3]
                pos_dist = np.linalg.norm(pos - np.array([0.3, 0, 0.3]))
                return topdown_score * 0.5 + conf * 1.0 - pos_dist * 0.1
            
            valid_grasps.sort(key=compute_presort_score, reverse=True)
            
            self.get_logger().info(f"Pre-sorted {len(valid_grasps)} grasps, solving IK one by one...")
            
            self.ik_solutions = []
            for orig_idx, grasp, conf in valid_grasps:
                position = grasp[:3, 3]
                orientation = R.from_matrix(grasp[:3, :3]).as_quat()
                
                joints = self.compute_ik(position, orientation, timeout_sec=1.0)
                
                if joints is not None:
                    joint_dist = self.compute_joint_distance(joints)
                    approach_dir = grasp[:3, 2]
                    topdown = -approach_dir[2]
                    combined_score = topdown * 0.3 + conf * 1.0 - joint_dist * 0.1
                    self.ik_solutions.append((orig_idx, grasp, conf, joints, combined_score))
                    
                    self.get_logger().info(f"Found IK solution (grasp #{len(self.ik_solutions)})")
                    
                    self.highlight_grasp_in_meshcat(orig_idx)
                    
                    self.current_grasp_index = 0
                    self.workflow_state = "user_selection"
                    self._prompt_user_selection_single(orig_idx, grasp, conf, joints, combined_score)
                    return
            
            if len(self.ik_solutions) == 0:
                self.get_logger().warn("No IK solutions found!")
                return
        
        elif self.workflow_state == "user_selection":
            pass
        
        elif self.workflow_state == "executing":
            pass
    
    def _prompt_user_selection_single(self, init_orig_idx, init_grasp, init_conf, init_joints, init_score):
        """Prompt user for a single grasp (non-blocking)."""
        import threading
        
        def user_input_thread():
            current_grasp = (init_orig_idx, init_grasp, init_conf, init_joints, init_score)
            
            while True:
                if current_grasp is None:
                    self.get_logger().info("Finding next IK solution...")
                    found_next = self._find_next_ik_solution()
                    if not found_next:
                        self.get_logger().info("No more IK solutions. Waiting for new grasp poses...")
                        self.clear_highlight_in_meshcat()
                        self.workflow_state = "wait_grasps"
                        return
                    current_grasp = self.ik_solutions[-1]
                
                orig_idx, grasp, conf, joints, score = current_grasp
                current_grasp = None
                
                self.highlight_grasp_in_meshcat(orig_idx)
                
                pos = grasp[:3, 3]
                approach_dir = grasp[:3, 2]
                print("\n" + "=" * 50)
                print(f"Grasp Candidate (score: {score:.3f})")
                print("=" * 50)
                print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                print(f"Approach: [{approach_dir[0]:.2f}, {approach_dir[1]:.2f}, {approach_dir[2]:.2f}]")
                print(f"Confidence: {conf:.3f}")
                print(f"Joint angles: {[f'{j:.2f}' for j in joints]}")
                print("=" * 50)
                
                try:
                    response = input("Execute? [y]es / [n]ext / [q]uit: ").strip().lower()
                except EOFError:
                    self.clear_highlight_in_meshcat()
                    self.workflow_state = "wait_grasps"
                    return
                
                if response in ['q', 'quit']:
                    self.get_logger().info("User quit")
                    self.clear_highlight_in_meshcat()
                    self.workflow_state = "wait_grasps"
                    return
                
                if response in ['n', 'next', 's', 'skip', '']:
                    self.get_logger().info("Skipping, finding next...")
                    continue
                
                if response in ['y', 'yes']:
                    success = self.execute_grasp(grasp)
                    
                    if success:
                        self.get_logger().info("Grasp successful!")
                        try:
                            next_action = input("Open gripper and return HOME? [y/n]: ").strip().lower()
                        except EOFError:
                            next_action = 'y'
                        
                        if next_action in ['y', 'yes', '']:
                            self.open_gripper()
                            time.sleep(0.5)
                            if not self.move_to_home():
                                self.get_logger().error("Failed to return to HOME position!")
                            else:
                                self.get_logger().info("Returned to HOME position successfully")
                        
                        self.clear_highlight_in_meshcat()
                        self.workflow_state = "wait_grasps"
                        self.get_logger().info("Waiting for new grasp poses...")
                        return
                    else:
                        self.get_logger().warn("Grasp failed!")
                        if not self.move_to_home():
                            self.get_logger().error("Failed to return to HOME position after failed grasp!")
                        continue
        
        input_thread = threading.Thread(target=user_input_thread, daemon=True)
        input_thread.start()
    
    def _find_next_ik_solution(self) -> bool:
        """Find the next valid IK solution from remaining grasps."""
        with self.grasp_lock:
            if self.grasp_poses is None:
                return False
            grasp_poses_cam = self.grasp_poses.copy()
            confidences = self.grasp_confidences.copy() if self.grasp_confidences is not None else None
        
        if grasp_poses_cam is None or len(grasp_poses_cam) == 0:
            return False
        
        tried_indices = set(sol[0] for sol in self.ik_solutions)
        
        grasps_base = []
        for i, grasp_cam in enumerate(grasp_poses_cam):
            if i in tried_indices:
                continue
            grasp_base = self.transform_grasp_to_base(grasp_cam)
            if grasp_base is not None:
                grasps_base.append((i, grasp_base, confidences[i] if confidences is not None else 0.5))
        
        def compute_presort_score(item):
            idx, grasp, conf = item
            approach_dir = grasp[:3, 2]
            topdown_score = -approach_dir[2]
            pos = grasp[:3, 3]
            pos_dist = np.linalg.norm(pos - np.array([0.3, 0, 0.3]))
            return topdown_score * 0.5 + conf * 1.0 - pos_dist * 0.1
        
        grasps_base.sort(key=compute_presort_score, reverse=True)
        
        for orig_idx, grasp, conf in grasps_base:
            z_min = getattr(self.args, 'min_z', 0.01)
            if grasp[2, 3] < z_min:
                continue
            approach_dir = grasp[:3, 2]
            if approach_dir[2] > 0.3:
                continue
            
            position = grasp[:3, 3]
            orientation = R.from_matrix(grasp[:3, :3]).as_quat()
            
            joints = self.compute_ik(position, orientation, timeout_sec=1.0)
            
            if joints is not None:
                joint_dist = self.compute_joint_distance(joints)
                topdown = -approach_dir[2]
                combined_score = topdown * 0.3 + conf * 1.0 - joint_dist * 0.1
                self.ik_solutions.append((orig_idx, grasp, conf, joints, combined_score))
                return True
        
        return False


def main():
    parser = argparse.ArgumentParser(description='Grasp Execution Node (D405)')
    parser.add_argument('--min_z', type=float, default=0.0,
                       help='Minimum Z height for valid grasps (m)')
    parser.add_argument('--offset_x', type=float, default=-0.0,
                       help='X offset correction in base frame (m)')
    parser.add_argument('--offset_y', type=float, default=-0.0,
                       help='Y offset correction in base frame (m)')
    parser.add_argument('--offset_z', type=float, default=0.0,
                       help='Z offset correction in base frame (m)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    node = GraspExecutionNode(args)
    
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
