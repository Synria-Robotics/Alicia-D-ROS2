#!/usr/bin/env python3
"""
Cube Sorting Node for Alicia D Robot

This node implements the cube sorting logic using MoveIt for robot control.
It subscribes to detected cube positions and executes pick-and-place operations
to sort cubes by color into designated drop zones.

Workflow:
1. Move to HOME position for detection
2. Wait for cube detection from cube_detection node
3. Pick up detected cubes by color priority (green first, then blue)
4. Place cubes at corresponding drop zones (GREEN -> DROP_POS_1, BLUE -> DROP_POS_2)
5. Repeat until all cubes are sorted

Usage:
1. Start the robot:
   ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm

2. Start the camera:
   ros2 launch orbbec_camera gemini_335.launch.py

3. Start cube detection (in another terminal):
   ros2 launch alicia_d_cube_sort cube_detection.launch.py

4. Start cube sorting (in another terminal):
   ros2 launch alicia_d_cube_sort cube_sorting.launch.py

Author: Synria Robotics
Date: 2026-01
"""

import os
import sys
import yaml
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    PositionIKRequest, RobotState, Constraints, JointConstraint,
    MotionPlanRequest, WorkspaceParameters
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory


class CubeSorter(Node):
    """
    Cube Sorting Node using MoveIt
    
    Controls the robot arm to sort cubes by color using predefined positions.
    """
    
    def __init__(self):
        super().__init__('cube_sorter')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Load configuration
        self.config = self._load_config()
        self._load_positions()
        
        # Callback groups for async operations
        self.action_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()
        
        # Create Action Clients for MoveIt control
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/Alicia_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group)
        
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/Gripper_controller/follow_joint_trajectory',
            callback_group=self.action_callback_group)
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.arm_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Action servers connected!')
        
        # MoveGroup action client for reliable motion planning and execution
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('MoveGroup action server connected!')
        
        # ExecuteTrajectory action client for executing planned trajectories
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for ExecuteTrajectory action server...')
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('ExecuteTrajectory action server connected!')
        
        # Motion planning service client
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for motion planning service...')
        self.plan_client.wait_for_service()
        self.get_logger().info('Motion planning service connected!')
        
        # Joint names
        self.arm_joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.gripper_joint_names = ['Gripper']
        
        # TF2 for coordinate transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # IK service client for computing joint positions from Cartesian pose
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.action_callback_group)
        self.get_logger().info('Waiting for IK service...')
        self.ik_client.wait_for_service()
        self.get_logger().info('IK service connected!')
        
        # Grasp offsets from config
        grasp_offsets = self.config.get('grasp_offsets', {})
        self.pre_grasp_z = grasp_offsets.get('pre_grasp_z', 0.05)
        self.grasp_z = grasp_offsets.get('grasp_z', 0.03)
        self.lift_z = grasp_offsets.get('lift_z', 0.05)
        
        # End effector orientation for top-down grasp (pointing down)
        # Quaternion from euler(0, pi, 0) - rotation 180 deg around Y axis
        # Add pi rotation around Z to align gripper properly
        # Note: Adjust Z rotation (last value) if gripper still rotates awkwardly
        grasp_rot = R.from_euler('xyz', [0, np.pi, np.pi])  # Y:180° + Z:180°
        quat = grasp_rot.as_quat()  # Returns [x, y, z, w]
        self.grasp_orientation = [quat[0], quat[1], quat[2], quat[3]]
        self.get_logger().info(f'Grasp orientation (xyzw): {self.grasp_orientation}')
        
        # Latest cube detections
        self.latest_cubes = {'green': [], 'blue': []}
        
        # Detection history for noise filtering (multi-frame confirmation)
        # Each entry: {'position': (x,y,z), 'count': int, 'last_seen': time}
        # Parameters are loaded from config in _load_positions()
        self.detection_history = {'green': [], 'blue': []}
        
        # Current joint state for IK seed
        self.current_joint_state = None
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
            callback_group=self.subscription_callback_group)
        
        # Subscribe to cube detection topics (use separate callback group to receive msgs during workflow)
        self.sub_cubes_green = self.create_subscription(
            PoseArray, '/vision/cubes/green', self._on_cubes_green, 10,
            callback_group=self.subscription_callback_group)
        self.sub_cubes_blue = self.create_subscription(
            PoseArray, '/vision/cubes/blue', self._on_cubes_blue, 10,
            callback_group=self.subscription_callback_group)
        
        # State machine
        self.workflow_state = "idle"
        self.current_cube_color = None
        self.current_cube_position = None
        self._moving_now = False
        self._detection_start_time = None  # For non-blocking detection wait
        
        # Timer for workflow execution
        self.workflow_timer = self.create_timer(0.5, self._workflow_tick)
        
        # Print startup info
        self._print_startup_info()
    
    def _declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('config_file', '')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
    
    def _get_parameters(self):
        """Get parameter values"""
        self.config_file = self.get_parameter('config_file').value
        self.auto_start = self.get_parameter('auto_start').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
    
    def _load_config(self):
        """Load configuration from YAML file"""
        config = {}
        
        if self.config_file and os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = yaml.safe_load(f) or {}
                self.get_logger().info(f'Loaded config from: {self.config_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config: {e}')
        else:
            # Try to find config in package share directory
            try:
                pkg_share = get_package_share_directory('alicia_d_cube_sort')
                default_config = os.path.join(pkg_share, 'config', 'cube_sorting.yaml')
                if os.path.exists(default_config):
                    with open(default_config, 'r') as f:
                        config = yaml.safe_load(f) or {}
                    self.get_logger().info(f'Loaded default config from: {default_config}')
            except Exception:
                pass
        
        return config
    
    def _load_positions(self):
        """Load robot positions from config"""
        positions = self.config.get('positions', {})
        
        # HOME position
        home = positions.get('home', {})
        self.home_position = home.get('joints', 
            [0.0, 0.1089, 0.6703, 0.0, -1.4174, 0.0])
        
        # Drop positions
        drop_green = positions.get('drop_green', {})
        self.drop_green_position = drop_green.get('joints',
            [1.0, -0.0483, 0.8309, -0.0545, -1.4830, -1.3617])
        
        drop_blue = positions.get('drop_blue', {})
        self.drop_blue_position = drop_blue.get('joints',
            [-1.0, 0.0882, 0.3399, 0.0468, -1.2804, -1.3249])
        
        # Color to drop position mapping
        self.color_drop_map = self.config.get('color_drop_map', {
            'green': 'drop_green',
            'blue': 'drop_blue'
        })
        
        # Gripper positions
        gripper = self.config.get('gripper', {})
        self.gripper_open = [gripper.get('open', 0.0)]
        self.gripper_closed = [gripper.get('closed', 0.024)]
        
        # Motion parameters
        motion = self.config.get('motion', {})
        self.arm_move_duration = motion.get('arm_move_duration', 2.0)
        self.gripper_duration = motion.get('gripper_duration', 1.0)
        self.detection_wait = motion.get('detection_wait', 2.0)
        
        # Detection confirmation parameters (noise filtering)
        detection_conf = self.config.get('detection_confirmation', {})
        self.detection_confirm_count = detection_conf.get('confirm_count', 5)
        self.detection_distance_threshold = detection_conf.get('distance_threshold', 0.01)
        self.detection_timeout = detection_conf.get('timeout', 2.0)
        
        # Grasp offsets
        grasp = self.config.get('grasp_offsets', {})
        self.pre_grasp_z = grasp.get('pre_grasp_z', 0.05)
        self.grasp_z = grasp.get('grasp_z', 0.01)
        self.lift_z = grasp.get('lift_z', 0.10)
        self.retract_distance = grasp.get('retract_distance', 0.05)  # Retract towards base while lifting
        
        # Sorting priority
        self.sorting_priority = self.config.get('sorting_priority', ['green', 'blue'])
    
    def _print_startup_info(self):
        """Print startup information"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Cube Sorting Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Auto Start: {self.auto_start}')
        self.get_logger().info(f'Base Frame: {self.base_frame}')
        self.get_logger().info(f'Camera Frame: {self.camera_frame}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Positions:')
        self.get_logger().info(f'  HOME: {self.home_position}')
        self.get_logger().info(f'  DROP_GREEN: {self.drop_green_position}')
        self.get_logger().info(f'  DROP_BLUE: {self.drop_blue_position}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Color Drop Mapping:')
        for color, drop_name in self.color_drop_map.items():
            self.get_logger().info(f'  {color} -> {drop_name}')
        self.get_logger().info('-' * 60)
        self.get_logger().info('Sorting Priority: ' + ' -> '.join(self.sorting_priority))
        self.get_logger().info('=' * 60)
        
        if self.auto_start:
            self.get_logger().info('Auto-starting workflow in 1 seconds...')
            time.sleep(1)
            self.workflow_state = "idle"
    
    def _on_cubes_green(self, msg: PoseArray):
        """Callback for green cube detections (positions already in base frame)"""
        detections = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.latest_cubes['green'] = detections
        self._update_detection_history('green', detections)
    
    def _on_cubes_blue(self, msg: PoseArray):
        """Callback for blue cube detections (positions already in base frame)"""
        detections = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.latest_cubes['blue'] = detections
        self._update_detection_history('blue', detections)
    
    def _update_detection_history(self, color, detections):
        """Update detection history for multi-frame confirmation"""
        current_time = time.time()
        history = self.detection_history[color]
        
        # Remove old entries that haven't been seen recently
        history[:] = [h for h in history 
                      if current_time - h['last_seen'] < self.detection_timeout]
        
        # Match current detections to history
        for det in detections:
            matched = False
            for h in history:
                dist = np.sqrt(
                    (det[0] - h['position'][0])**2 +
                    (det[1] - h['position'][1])**2 +
                    (det[2] - h['position'][2])**2
                )
                if dist < self.detection_distance_threshold:
                    # Update existing entry with averaged position
                    alpha = 0.3  # Smoothing factor
                    h['position'] = (
                        h['position'][0] * (1-alpha) + det[0] * alpha,
                        h['position'][1] * (1-alpha) + det[1] * alpha,
                        h['position'][2] * (1-alpha) + det[2] * alpha
                    )
                    h['count'] += 1
                    h['last_seen'] = current_time
                    matched = True
                    break
            
            if not matched:
                # New detection, add to history
                history.append({
                    'position': det,
                    'count': 1,
                    'last_seen': current_time
                })
    
    def _get_confirmed_cubes(self, color):
        """Get cubes that have been confirmed by multiple detections"""
        confirmed = []
        for h in self.detection_history[color]:
            if h['count'] >= self.detection_confirm_count:
                confirmed.append(h['position'])
        return confirmed
    
    def _on_joint_state(self, msg: JointState):
        """Callback for joint state updates"""
        self.current_joint_state = msg
    
    def _transform_to_base(self, position_camera):
        """Transform position from camera frame to base frame"""
        try:
            # Get transform from camera to base
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            
            # Apply transformation
            t = transform.transform.translation
            r = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            rot = R.from_quat([r.x, r.y, r.z, r.w])
            
            # Transform point
            p_camera = np.array(position_camera)
            p_base = rot.apply(p_camera) + np.array([t.x, t.y, t.z])
            
            return tuple(p_base)
            
        except TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None
    
    def compute_ik(self, position, orientation=None):
        """
        Compute inverse kinematics for a target pose.
        
        Args:
            position: (x, y, z) tuple in base frame
            orientation: (qx, qy, qz, qw) quaternion, defaults to top-down grasp
            
        Returns:
            List of joint positions or None if IK failed
        """
        if orientation is None:
            orientation = self.grasp_orientation
        
        self.get_logger().info(f'Computing IK for position: ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})')
        self.get_logger().info(f'Orientation (xyzw): ({orientation[0]:.4f}, {orientation[1]:.4f}, {orientation[2]:.4f}, {orientation[3]:.4f})')
        
        # Build IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = "Alicia"  # MoveIt group name from SRDF
        request.ik_request.avoid_collisions = False  # Disable collision check for faster IK
        request.ik_request.timeout.sec = 5
        request.ik_request.timeout.nanosec = 0
        
        # Set the end effector link name
        request.ik_request.ik_link_name = "gripper_center"
        
        # Use current joint state as seed for IK solver
        if self.current_joint_state is not None:
            request.ik_request.robot_state.joint_state = self.current_joint_state
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(position[0])
        pose_stamped.pose.position.y = float(position[1])
        pose_stamped.pose.position.z = float(position[2])
        pose_stamped.pose.orientation.x = float(orientation[0])
        pose_stamped.pose.orientation.y = float(orientation[1])
        pose_stamped.pose.orientation.z = float(orientation[2])
        pose_stamped.pose.orientation.w = float(orientation[3])
        request.ik_request.pose_stamped = pose_stamped
        
        # Call IK service
        try:
            future = self.ik_client.call_async(request)
            
            # Wait for result with timeout
            timeout_sec = 5.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('IK service call timeout')
                    return None
                time.sleep(0.05)
            
            response = future.result()
            
            if response.error_code.val == response.error_code.SUCCESS:
                # Extract joint positions for arm joints
                joint_positions = []
                for joint_name in self.arm_joint_names:
                    if joint_name in response.solution.joint_state.name:
                        idx = response.solution.joint_state.name.index(joint_name)
                        joint_positions.append(response.solution.joint_state.position[idx])
                
                if len(joint_positions) == len(self.arm_joint_names):
                    self.get_logger().info(f'IK solution found: {[f"{j:.3f}" for j in joint_positions]}')
                    return joint_positions
                else:
                    self.get_logger().error('IK solution incomplete')
                    return None
            else:
                # More detailed error message
                error_codes = {
                    -1: "PLANNING_FAILED",
                    -2: "INVALID_MOTION_PLAN",
                    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -4: "CONTROL_FAILED",
                    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -6: "TIMED_OUT",
                    -7: "PREEMPTED",
                    -10: "START_STATE_IN_COLLISION",
                    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                    -12: "GOAL_IN_COLLISION",
                    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                    -14: "GOAL_CONSTRAINTS_VIOLATED",
                    -15: "INVALID_GROUP_NAME",
                    -16: "INVALID_GOAL_CONSTRAINTS",
                    -17: "INVALID_ROBOT_STATE",
                    -18: "INVALID_LINK_NAME",
                    -19: "INVALID_OBJECT_NAME",
                    -31: "NO_IK_SOLUTION",
                    99999: "FAILURE",
                }
                error_name = error_codes.get(response.error_code.val, "UNKNOWN")
                self.get_logger().warn(f'IK failed with error code: {response.error_code.val} ({error_name})')
                self.get_logger().warn(f'  Target position: ({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})')
                return None
                
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')
            return None
    
    def move_to_pose(self, position, orientation=None, duration_sec=None):
        """
        Move arm to a Cartesian pose using IK.
        
        Args:
            position: (x, y, z) tuple in base frame
            orientation: (qx, qy, qz, qw) quaternion
            duration_sec: Motion duration
            
        Returns:
            True if successful, False otherwise
        """
        joint_positions = self.compute_ik(position, orientation)
        if joint_positions is None:
            return False
        
        return self.move_arm_to_joint_positions(joint_positions, duration_sec)
    
    def pick_cube(self, cube_position):
        """
        Execute pick operation for a cube at the given position.
        
        Args:
            cube_position: (x, y, z) tuple in base frame
            
        Returns:
            True if successful, False otherwise
        """
        x, y, z = cube_position
        
        # Step 1: Move to pre-grasp position (above the cube)
        pre_grasp_pos = (x, y, z + self.pre_grasp_z)
        self.get_logger().info(f'Moving to pre-grasp: {pre_grasp_pos}')
        if not self.move_to_pose(pre_grasp_pos):
            self.get_logger().error('Failed to reach pre-grasp position')
            return False
        time.sleep(0.5)  # Wait for arm to settle
        
        # Step 2: Open gripper (arm is now at pre-grasp)
        if not self.open_gripper():
            self.get_logger().warn('Gripper open command may have failed')
        time.sleep(0.5)  # Wait for gripper to fully open
        
        # Step 3: Move down to grasp position
        grasp_pos = (x, y, z + self.grasp_z)
        self.get_logger().info(f'Moving to grasp: {grasp_pos}')
        if not self.move_to_pose(grasp_pos, duration_sec=1.5):
            self.get_logger().error('Failed to reach grasp position')
            return False
        time.sleep(0.5)  # Wait for arm to fully settle before grasping
        
        # Step 4: Close gripper to grasp cube (arm is now at grasp position)
        self.get_logger().info('Closing gripper to grasp cube...')
        if not self.close_gripper():
            self.get_logger().warn('Gripper close command may have failed')
        time.sleep(1.0)  # Wait longer for gripper to fully close and grip cube
        
        # Step 5: Retract and lift the cube (reduce moment arm on Joint 2)
        # Calculate retract direction (towards base origin to reduce horizontal distance)
        horizontal_dist = np.sqrt(x**2 + y**2)
        if horizontal_dist > 0.01:  # Avoid division by zero
            # Retract vector: move towards base origin
            retract_x = -x / horizontal_dist * self.retract_distance
            retract_y = -y / horizontal_dist * self.retract_distance
        else:
            retract_x = 0.0
            retract_y = 0.0
        
        lift_pos = (x + retract_x, y + retract_y, z + self.lift_z)
        self.get_logger().info(f'Retract & lift to: {lift_pos} (retracted {self.retract_distance:.3f}m towards base)')
        if not self.move_to_pose(lift_pos, duration_sec=1.5):
            self.get_logger().error('Failed to retract and lift cube')
            return False
        
        self.get_logger().info('Cube picked successfully!')
        return True

    def plan_to_joint_positions(self, joint_positions):
        """
        Plan a trajectory to the specified joint positions using MoveIt motion planning.
        
        Args:
            joint_positions: List of target joint positions
            
        Returns:
            Planned trajectory or None if planning failed
        """
        request = GetMotionPlan.Request()
        
        # Set group name
        request.motion_plan_request.group_name = "Alicia"
        
        # Set number of planning attempts and allowed time
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        
        # Set velocity and acceleration scaling
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        
        # Use current state as start state
        request.motion_plan_request.start_state.is_diff = True
        
        # Set goal constraints (joint space)
        goal_constraints = Constraints()
        goal_constraints.name = "joint_goal"
        
        for i, joint_name in enumerate(self.arm_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_positions[i])
            joint_constraint.tolerance_above = 0.01  # ~0.5 degrees
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        request.motion_plan_request.goal_constraints.append(goal_constraints)
        
        # Call planning service
        try:
            self.get_logger().info(f'Planning to joint positions: {[f"{j:.3f}" for j in joint_positions]}')
            future = self.plan_client.call_async(request)
            
            # Wait for result with timeout
            timeout_sec = 10.0
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('Motion planning service timeout')
                    return None
                time.sleep(0.05)
            
            response = future.result()
            
            if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f'Motion planning succeeded, trajectory has {len(response.motion_plan_response.trajectory.joint_trajectory.points)} points')
                return response.motion_plan_response.trajectory
            else:
                error_codes = {
                    1: "SUCCESS",
                    -1: "PLANNING_FAILED",
                    -2: "INVALID_MOTION_PLAN",
                    -4: "CONTROL_FAILED",
                    -6: "TIMED_OUT",
                    -10: "START_STATE_IN_COLLISION",
                    -12: "GOAL_IN_COLLISION",
                    -15: "INVALID_GROUP_NAME",
                    -31: "NO_IK_SOLUTION",
                    99999: "FAILURE",
                }
                error_name = error_codes.get(response.motion_plan_response.error_code.val, "UNKNOWN")
                self.get_logger().error(f'Motion planning failed: {response.motion_plan_response.error_code.val} ({error_name})')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Motion planning service call failed: {e}')
            return None
    
    def execute_trajectory(self, trajectory):
        """
        Execute a planned trajectory using the ExecuteTrajectory action.
        
        Args:
            trajectory: RobotTrajectory message to execute
            
        Returns:
            True if execution succeeded, False otherwise
        """
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('Executing trajectory...')
        
        # Send goal asynchronously
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance with timeout
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for trajectory execution goal acceptance')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution goal rejected')
            return False
        
        self.get_logger().info('Trajectory execution goal accepted, waiting for completion...')
        
        # Wait for result - calculate timeout based on trajectory duration
        if trajectory.joint_trajectory.points:
            last_point = trajectory.joint_trajectory.points[-1]
            traj_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            timeout_sec = traj_duration + 15.0  # Extra buffer
        else:
            timeout_sec = 30.0
        
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for trajectory execution result')
                return False
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Trajectory execution completed successfully')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error: {result.result.error_code.val}')
            return False
    
    def move_arm_with_moveit(self, joint_positions):
        """
        Move arm using MoveIt plan + execute (more reliable than direct trajectory).
        
        Args:
            joint_positions: List of target joint positions
            
        Returns:
            True if movement succeeded, False otherwise
        """
        # Plan trajectory
        trajectory = self.plan_to_joint_positions(joint_positions)
        if trajectory is None:
            self.get_logger().error('Failed to plan trajectory')
            return False
        
        # Execute trajectory
        success = self.execute_trajectory(trajectory)
        if success:
            # Wait for arm to physically settle
            time.sleep(0.5)
        return success

    def move_arm_to_joint_positions(self, joint_positions, duration_sec=None):
        """Move arm to specified joint positions using MoveIt plan+execute"""
        # First try MoveIt plan+execute (more reliable)
        self.get_logger().info(f'Moving arm to: {[f"{j:.3f}" for j in joint_positions]} using MoveIt')
        success = self.move_arm_with_moveit(joint_positions)
        
        if success:
            # Verify we actually reached the target
            if self._verify_joint_positions(joint_positions, tolerance=0.05):
                return True
            else:
                self.get_logger().warn('MoveIt reported success but position verification failed')
        
        # Fallback to direct trajectory control
        self.get_logger().warn('MoveIt plan+execute failed, trying direct trajectory control...')
        success = self._move_arm_direct(joint_positions, duration_sec)
        
        if success:
            # Wait and verify position
            time.sleep(1.0)
            if self._verify_joint_positions(joint_positions, tolerance=0.1):
                return True
            else:
                self.get_logger().error('Direct control also failed position verification')
                return False
        return False
    
    def _verify_joint_positions(self, target_positions, tolerance=0.05):
        """
        Verify that current joint positions match target positions.
        
        Args:
            target_positions: Expected joint positions
            tolerance: Maximum allowed error in radians
            
        Returns:
            True if within tolerance, False otherwise
        """
        if self.current_joint_state is None:
            self.get_logger().warn('No current joint state available for verification')
            return True  # Assume success if we can't verify
        
        # Extract current positions for arm joints
        current_positions = []
        for joint_name in self.arm_joint_names:
            if joint_name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(joint_name)
                current_positions.append(self.current_joint_state.position[idx])
            else:
                self.get_logger().warn(f'Joint {joint_name} not found in current state')
                return True  # Can't verify, assume success
        
        if len(current_positions) != len(target_positions):
            return True  # Can't verify, assume success
        
        # Check each joint
        max_error = 0.0
        for i, (current, target) in enumerate(zip(current_positions, target_positions)):
            error = abs(current - target)
            max_error = max(max_error, error)
            if error > tolerance:
                self.get_logger().warn(
                    f'Joint {self.arm_joint_names[i]} position error: {error:.4f} rad '
                    f'(current: {current:.4f}, target: {target:.4f})')
        
        if max_error <= tolerance:
            self.get_logger().info(f'Position verification passed (max error: {max_error:.4f} rad)')
            return True
        else:
            self.get_logger().error(f'Position verification failed (max error: {max_error:.4f} rad > tolerance {tolerance})')
            return False
    
    def _move_arm_direct(self, joint_positions, duration_sec=None):
        """Move arm using direct FollowJointTrajectory action (fallback method)"""
        if duration_sec is None:
            duration_sec = self.arm_move_duration
        
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'Moving arm to: {[f"{j:.3f}" for j in joint_positions]}')
        
        # Send goal asynchronously
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance with timeout
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for arm goal acceptance')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False
        
        self.get_logger().info('Arm goal accepted, waiting for result...')
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        timeout_sec = duration_sec + 10.0
        start_time = time.time()
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for arm motion result')
                return False
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Arm motion command completed')
            # Wait additional time for arm to physically settle
            time.sleep(0.3)
            return True
        else:
            self.get_logger().error(f'Arm motion failed, error: {result.result.error_code}')
            return False
    
    def move_gripper(self, gripper_position, duration_sec=None):
        """Control gripper (blocking with timeout)"""
        if duration_sec is None:
            duration_sec = self.gripper_duration
        
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(gripper_position)
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        gripper_state = "closing" if gripper_position[0] > 0.01 else "opening"
        self.get_logger().info(f'Gripper {gripper_state}...')
        
        # Send goal asynchronously
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance with timeout
        timeout_sec = 5.0
        start_time = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for gripper goal acceptance')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False
        
        self.get_logger().info('Gripper goal accepted, waiting for result...')
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        timeout_sec = duration_sec + 5.0
        start_time = time.time()
        while not result_future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for gripper result')
                return False
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info(f'Gripper {gripper_state} complete')
            time.sleep(0.3)  # Wait for gripper to physically settle
            return True
        else:
            self.get_logger().error(f'Gripper motion failed, error: {result.result.error_code}')
            return False
    
    def open_gripper(self):
        """Open the gripper"""
        return self.move_gripper(self.gripper_open, duration_sec=1.5)
    
    def close_gripper(self):
        """Close the gripper"""
        return self.move_gripper(self.gripper_closed, duration_sec=1.5)
    
    def move_to_home(self):
        """Move to HOME position"""
        self.get_logger().info('Moving to HOME position...')
        return self.move_arm_to_joint_positions(self.home_position, 3.0)  # Longer duration for reliable movement
    
    def move_to_drop_zone(self, color):
        """Move to drop zone for specified color"""
        drop_name = self.color_drop_map.get(color, 'drop_green')
        
        if drop_name == 'drop_green':
            position = self.drop_green_position
        elif drop_name == 'drop_blue':
            position = self.drop_blue_position
        else:
            position = self.drop_green_position
        
        self.get_logger().info(f'Moving to {drop_name} for {color} cube...')
        return self.move_arm_to_joint_positions(position, 3.0)  # Use longer duration for drop zone movement
    
    def _clear_detections(self):
        """Clear all cube detections and history"""
        self.latest_cubes = {'green': [], 'blue': []}
        self.detection_history = {'green': [], 'blue': []}
        self.get_logger().info('Cleared old cube detections and history')
    
    def _select_cube(self):
        """Select a confirmed cube to pick based on priority"""
        for color in self.sorting_priority:
            confirmed = self._get_confirmed_cubes(color)
            if confirmed:
                # Select the first confirmed cube
                pos_base = confirmed[0]
                
                # Remove this cube from history to prevent re-picking
                self.detection_history[color] = [
                    h for h in self.detection_history[color]
                    if np.sqrt(
                        (h['position'][0] - pos_base[0])**2 +
                        (h['position'][1] - pos_base[1])**2 +
                        (h['position'][2] - pos_base[2])**2
                    ) > self.detection_distance_threshold
                ]
                
                self.get_logger().info(
                    f'Selected CONFIRMED {color} cube at base frame: '
                    f'x={pos_base[0]:.3f}, y={pos_base[1]:.3f}, z={pos_base[2]:.3f}')
                return color, pos_base
        
        return None, None
    
    def _workflow_tick(self):
        """State machine tick"""
        if self._moving_now:
            return
        
        if self.workflow_state == "idle":
            # Start workflow
            self.get_logger().info('=' * 40)
            self.get_logger().info('Starting cube sorting workflow')
            self.get_logger().info('=' * 40)
            
            self._moving_now = True
            try:
                self._clear_detections()
                self.open_gripper()
                time.sleep(0.5)
                self.move_to_home()
                self.workflow_state = "detecting"
                self._detection_start_time = self.get_clock().now()  # Start timing
                self.get_logger().info('At HOME position, waiting for cube detection...')
            finally:
                self._moving_now = False
        
        elif self.workflow_state == "detecting":
            # Non-blocking wait for detection (allow subscription callbacks to run)
            if self._detection_start_time is not None:
                elapsed = (self.get_clock().now() - self._detection_start_time).nanoseconds / 1e9
                if elapsed < self.detection_wait:
                    return  # Still waiting, let other callbacks run
                self._detection_start_time = None  # Done waiting
            
            color, position = self._select_cube()
            
            if color is not None and position is not None:
                self.current_cube_color = color
                self.current_cube_position = position
                self.workflow_state = "picking"
                self.get_logger().info(f'Found CONFIRMED {color} cube to pick at {position}')
            else:
                # Check detection status
                total_raw = sum(len(v) for v in self.latest_cubes.values())
                total_confirmed = sum(len(self._get_confirmed_cubes(c)) for c in self.sorting_priority)
                
                if total_raw > 0 and total_confirmed == 0:
                    # Detections exist but not yet confirmed - keep waiting
                    self.get_logger().info(
                        f'Detected {total_raw} cubes, waiting for confirmation '
                        f'({self.detection_confirm_count} frames needed)...')
                    self._detection_start_time = self.get_clock().now()
                elif total_raw == 0:
                    self.get_logger().info('No cubes detected, waiting...')
                    self._detection_start_time = self.get_clock().now()
                # Stay in detecting state
        
        elif self.workflow_state == "picking":
            # Execute pick operation using IK
            self._moving_now = True
            try:
                self.get_logger().info(f'Picking {self.current_cube_color} cube at position: '
                                       f'x={self.current_cube_position[0]:.3f}, '
                                       f'y={self.current_cube_position[1]:.3f}, '
                                       f'z={self.current_cube_position[2]:.3f}')
                
                # Execute pick sequence: pre-grasp -> grasp -> lift
                success = self.pick_cube(self.current_cube_position)
                
                if success:
                    self.workflow_state = "placing"
                    self.get_logger().info('Cube grasped successfully, moving to drop zone...')
                else:
                    self.get_logger().error('Pick operation failed, returning to detect...')
                    self.open_gripper()
                    self.move_to_home()
                    self.workflow_state = "detecting"
                    self._detection_start_time = self.get_clock().now()
            finally:
                self._moving_now = False
        
        elif self.workflow_state == "placing":
            # Execute place operation
            self._moving_now = True
            try:
                # Move to drop zone
                self.get_logger().info(f'Moving to drop zone for {self.current_cube_color}...')
                self.move_to_drop_zone(self.current_cube_color)
                time.sleep(0.5)  # Wait for arm to fully settle at drop zone
                
                # Release cube (arm is now at drop position)
                self.open_gripper()
                time.sleep(0.5)  # Wait for gripper to fully open
                
                self.get_logger().info(f'Dropped {self.current_cube_color} cube at drop zone')
                
                # Clear current cube info
                self.current_cube_color = None
                self.current_cube_position = None
                
                # Return to home position for next detection cycle
                self.get_logger().info('Returning to HOME position...')
                success = self.move_to_home()
                time.sleep(0.5)  # Wait for arm to settle at home
                
                if success:
                    self.get_logger().info('Returned to HOME, starting new detection cycle...')
                else:
                    self.get_logger().warn('Failed to return to HOME, retrying...')
                    self.move_to_home()
                    time.sleep(0.5)
                
                # Clear detection history and start fresh detection
                self._clear_detections()
                self._detection_start_time = self.get_clock().now()
                self.workflow_state = "detecting"
            finally:
                self._moving_now = False
    
    def run_sorting_cycle(self):
        """Run a complete sorting cycle manually"""
        self.get_logger().info('Starting manual sorting cycle...')
        
        # Move to home
        self.open_gripper()
        self.move_to_home()
        
        # Wait for detections
        self.get_logger().info('Waiting for cube detections...')
        time.sleep(self.detection_wait)
        
        # Process all detected cubes
        while True:
            color, position = self._select_cube()
            
            if color is None:
                self.get_logger().info('No more cubes to sort')
                break
            
            self.get_logger().info(f'Sorting {color} cube at {position}')
            
            # Pick
            self.close_gripper()
            time.sleep(0.5)
            
            # Place
            self.move_to_drop_zone(color)
            self.open_gripper()
            time.sleep(0.5)
            
            # Return home for next detection
            self.move_to_home()
            time.sleep(self.detection_wait)
        
        self.get_logger().info('Sorting cycle complete!')
        return True


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = CubeSorter()
    
    # Use MultiThreadedExecutor to allow concurrent callback processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
