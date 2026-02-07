#!/usr/bin/env python3
"""
Grasp Generation - GraspGen
Environment: GraspGen

Subscribes to point cloud and 2D mask, projects mask to 3D, extracts object points,
runs GraspGen inference, and publishes grasp poses with meshcat visualization.

Usage:
    # Make sure meshcat-server is running in another terminal
    conda activate GraspGen
    python grasp_generation.py [options]
"""

import os
import sys
import argparse
import time
import threading
import struct
import json
import numpy as np
import torch
import logging
import cv2
from collections import deque

# Add GraspGen to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GRASPGEN_DIR = os.path.join(SCRIPT_DIR, '..', '..', 'Models', 'GraspGen')
sys.path.insert(0, GRASPGEN_DIR)

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
    from geometry_msgs.msg import PoseArray, Pose
    from std_msgs.msg import Header, Float32MultiArray, Int32
    import message_filters
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARNING] ROS 2 Python bindings not available. Running in standalone mode.")

# GraspGen imports
from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.utils.point_cloud_utils import point_cloud_outlier_removal
from grasp_gen.utils.meshcat_utils import (
    create_visualizer,
    get_color_from_score,
    visualize_grasp,
    visualize_pointcloud,
    make_frame
)

from scipy.spatial.transform import Rotation as R

# Local utilities
from utils.camera_utils import (
    CameraParameterManager,
    DEFAULT_IR_K, DEFAULT_RGB_K, DEFAULT_T_IR_TO_RGB,
    DEFAULT_IR_RESOLUTION, DEFAULT_RGB_RESOLUTION, DEFAULT_BASELINE
)


class GraspGenerationNode:
    """
    ROS 2 Node for GraspGen grasp pose generation.
    """
    
    def __init__(self, args):
        self.args = args
        self.grasp_sampler = None
        self.grasp_cfg = None
        self.vis = None
        
        # Camera parameter manager - will be initialized after ROS node creation
        # Use defaults initially, then update from ROS when available
        self.cam_manager = None
        
        # Camera intrinsics and extrinsics (will be updated from ROS)
        # Using defaults as fallback
        self.K_ir = DEFAULT_IR_K.copy()
        self.K_rgb = DEFAULT_RGB_K.copy()
        self.ir_resolution = DEFAULT_IR_RESOLUTION
        self.rgb_resolution = DEFAULT_RGB_RESOLUTION
        self.T_ir_to_rgb = DEFAULT_T_IR_TO_RGB.copy()
        
        # Use IR intrinsics for general point cloud operations
        self.K = self.K_ir
        self.camera_info_received = False  # Will be set to True after fetching from ROS
        
        # Data buffers
        self.pointcloud = None
        self.pointcloud_colors = None
        self.depth_image = None
        self.mask = None
        self.data_lock = threading.Lock()
        self.new_data_available = False
        
        # Grasp results
        self.grasp_poses = None
        self.grasp_confidences = None
        self.grasp_lock = threading.Lock()
        
        # 高亮索引 (-1 表示无高亮)
        self.highlight_index = -1
        self.highlight_lock = threading.Lock()
        self.highlight_pending = False  # 标记是否有待处理的高亮更新
        
        # 保存用于可视化的居中数据
        self.vis_pc_center = None
        self.vis_grasps_centered = None
        self.vis_confidences = None
        
        # Statistics
        self.inference_times = deque(maxlen=10)
        
        # Initialize model
        self._load_model()
        
        # Initialize meshcat visualizer
        self._init_meshcat()
        
        # Initialize ROS if available
        if ROS_AVAILABLE:
            self._init_ros()
    
    def _load_model(self):
        """Load GraspGen model."""
        logging.info("Loading GraspGen model...")
        
        gripper_config = self.args.gripper_config
        if not os.path.exists(gripper_config):
            raise FileNotFoundError(f"Gripper config not found: {gripper_config}")
        
        self.grasp_cfg = load_grasp_cfg(gripper_config)
        self.gripper_name = self.grasp_cfg.data.gripper_name
        
        self.grasp_sampler = GraspGenSampler(self.grasp_cfg)
        
        logging.info(f"GraspGen model loaded! Gripper: {self.gripper_name}")
    
    def _init_meshcat(self):
        """Initialize meshcat visualizer."""
        try:
            logging.info("Connecting to meshcat server...")
            self.vis = create_visualizer(clear=True)
            logging.info("Meshcat visualizer connected!")
        except Exception as e:
            logging.error(f"Failed to connect to meshcat: {e}")
            logging.error("Make sure meshcat-server is running: `meshcat-server`")
            self.vis = None
    
    def _init_ros(self):
        """Initialize ROS 2 node and subscribers/publishers."""
        rclpy.init()
        self.node = rclpy.create_node('grasp_generation_node')
        
        # Initialize camera parameter manager to fetch params from ROS
        self.cam_manager = CameraParameterManager(self.node, use_defaults=False)
        
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
        
        # Point cloud subscriber
        self.pc_sub = self.node.create_subscription(
            PointCloud2, '/grasp_6d/pointcloud',
            self._pointcloud_callback, sensor_qos)
        
        # Mask subscriber
        self.mask_sub = self.node.create_subscription(
            Image, '/grasp_6d/mask',
            self._mask_callback, sensor_qos)
        
        # Depth image subscriber (from camera)
        self.depth_sub = self.node.create_subscription(
            Image, '/camera/depth/image_raw',
            self._depth_callback, sensor_qos)
        
        # Grasp poses publisher
        self.grasp_pub = self.node.create_publisher(
            PoseArray, '/grasp_6d/grasp_poses', reliable_qos)
        
        # Grasp confidences publisher
        self.conf_pub = self.node.create_publisher(
            Float32MultiArray, '/grasp_6d/grasp_confidences', reliable_qos)
        
        # 高亮索引订阅 - grasp_execution.py 发布要高亮的夹爪索引
        self.highlight_sub = self.node.create_subscription(
            Int32, '/grasp_6d/highlight_index',
            self._highlight_callback, reliable_qos)
        
        logging.info("ROS 2 node initialized")
        logging.info("Subscribing to: /grasp_6d/pointcloud, /grasp_6d/mask, /grasp_6d/highlight_index")
        logging.info("Publishing to: /grasp_6d/grasp_poses")
    
    def _update_camera_params(self):
        """Update camera parameters from CameraParameterManager."""
        if self.cam_manager is not None and self.cam_manager.is_ready:
            self.K_ir = self.cam_manager.K_ir
            self.K_rgb = self.cam_manager.K_rgb
            self.ir_resolution = self.cam_manager.ir_resolution
            self.rgb_resolution = self.cam_manager.rgb_resolution
            self.T_ir_to_rgb = self.cam_manager.T_ir_to_rgb
            self.K = self.K_ir
            self.camera_info_received = True
            return True
        return False
    
    def _pointcloud_callback(self, msg: 'PointCloud2'):
        """Process incoming point cloud."""
        points, colors = self._parse_pointcloud2(msg)
        
        with self.data_lock:
            self.pointcloud = points
            self.pointcloud_colors = colors
            self.new_data_available = True
    
    def _mask_callback(self, msg: 'Image'):
        """Process incoming mask."""
        if msg.encoding == 'mono8':
            mask_raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            mask = mask_raw > 0  # Any non-zero value is True
            non_zero = np.sum(mask)
        else:
            logging.warning(f"Unsupported mask encoding: {msg.encoding}")
            return
        
        if non_zero > 0:
            with self.data_lock:
                self.mask = mask
                self.new_data_available = True
                if not getattr(self, '_mask_received_logged', False):
                    logging.info(f"Mask received: shape={mask.shape}, {non_zero} pixels")
                    self._mask_received_logged = True
    
    def _depth_callback(self, msg: 'Image'):
        """Process incoming depth image."""
        if msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth = depth.astype(np.float32) / 1000.0  # Convert to meters
        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            logging.warning(f"Unsupported depth encoding: {msg.encoding}")
            return
        
        with self.data_lock:
            self.depth_image = depth
    
    def _highlight_callback(self, msg: 'Int32'):
        """处理高亮索引更新。"""
        new_index = msg.data
        with self.highlight_lock:
            old_index = self.highlight_index
            self.highlight_index = new_index
            # 标记有待处理的高亮更新
            self.highlight_pending = True
        
        # 如果索引变化，尝试更新可视化
        if new_index != old_index:
            self._update_highlight_visualization()
    
    def _update_highlight_visualization(self):
        """更新 meshcat 中的高亮显示。"""
        if self.vis is None:
            return
        
        with self.highlight_lock:
            highlight_idx = self.highlight_index
        
        # 获取保存的可视化数据
        if self.vis_grasps_centered is None or self.vis_confidences is None:
            # 数据还没准备好，保持 pending 状态，等 visualize_in_meshcat 完成后处理
            logging.debug("Highlight data not ready yet, will apply when visualization completes")
            return
        
        try:
            grasps_centered = self.vis_grasps_centered
            confidences = self.vis_confidences
            
            # 重新绘制所有夹爪，高亮的用红色
            colors = get_color_from_score(confidences, use_255_scale=True)
            
            for i, (grasp, conf, color) in enumerate(zip(grasps_centered, confidences, colors)):
                # 如果是高亮索引，使用红色
                if i == highlight_idx:
                    grasp_color = [255, 0, 0]  # 红色
                    linewidth = 2.4  # 加粗
                else:
                    grasp_color = color.tolist()
                    linewidth = 0.6
                
                visualize_grasp(
                    self.vis,
                    f"grasps/{i:03d}",
                    grasp,
                    color=grasp_color,
                    gripper_name=self.gripper_name,
                    linewidth=linewidth
                )
            
            # 成功更新，清除 pending 标志
            with self.highlight_lock:
                self.highlight_pending = False
            
            if highlight_idx >= 0:
                logging.debug(f"Highlighted grasp #{highlight_idx} in meshcat")
            else:
                logging.debug("Cleared grasp highlight in meshcat")
                
        except Exception as e:
            logging.error(f"Failed to update highlight visualization: {e}")
    
    def _parse_pointcloud2(self, msg: 'PointCloud2'):
        """Parse PointCloud2 message to numpy arrays."""
        # Find field offsets
        field_names = {f.name: f.offset for f in msg.fields}
        
        points = []
        colors = []
        
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            
            # Extract XYZ
            x = struct.unpack_from('f', msg.data, offset + field_names['x'])[0]
            y = struct.unpack_from('f', msg.data, offset + field_names['y'])[0]
            z = struct.unpack_from('f', msg.data, offset + field_names['z'])[0]
            
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and z > 0:
                points.append([x, y, z])
                
                # Extract RGB if available
                if 'rgb' in field_names:
                    rgb_packed = struct.unpack_from('I', msg.data, offset + field_names['rgb'])[0]
                    r = (rgb_packed >> 16) & 0xFF
                    g = (rgb_packed >> 8) & 0xFF
                    b = rgb_packed & 0xFF
                    colors.append([r, g, b])
                else:
                    colors.append([128, 128, 128])
        
        return np.array(points, dtype=np.float32), np.array(colors, dtype=np.uint8)
    
    def extract_object_points(self, pointcloud: np.ndarray, colors: np.ndarray,
                             mask: np.ndarray, depth_image: np.ndarray = None):
        """
        Extract 3D points corresponding to the 2D mask.
        
        Strategy: 
        1. Transform point cloud from IR frame to RGB frame using extrinsic calibration
        2. Project transformed points to RGB image using RGB camera intrinsics
        3. Match against RGB mask
        
        Args:
            pointcloud: Full scene point cloud (N, 3) in IR camera frame
            colors: Point colors (N, 3)
            mask: 2D binary mask (H, W) - from RGB camera
            depth_image: Optional depth image (H, W) in meters
            
        Returns:
            object_points: Extracted object points (M, 3) in IR camera frame
            object_colors: Extracted colors (M, 3)
        """
        H, W = mask.shape  # RGB mask resolution (e.g., 720x1280)
        rgb_H, rgb_W = self.rgb_resolution  # Expected RGB resolution
        
        logging.debug(f"Mask shape: {mask.shape}, RGB resolution: {self.rgb_resolution}")
        logging.debug(f"Point cloud shape: {pointcloud.shape}")
        
        # Handle mask resolution mismatch
        if (H, W) != (rgb_H, rgb_W):
            logging.info(f"Resizing mask from {(H, W)} to {(rgb_H, rgb_W)}")
            mask = cv2.resize(mask.astype(np.uint8), (rgb_W, rgb_H), 
                             interpolation=cv2.INTER_NEAREST) > 0
            H, W = rgb_H, rgb_W
        
        # Step 1: Transform point cloud from IR frame to RGB frame
        # pointcloud: (N, 3), we need to apply T_ir_to_rgb (4x4 transformation)
        N = len(pointcloud)
        points_homo = np.hstack([pointcloud, np.ones((N, 1))])  # (N, 4)
        points_rgb = (self.T_ir_to_rgb @ points_homo.T).T[:, :3]  # (N, 3)
        
        # Step 2: Project to RGB image coordinates using RGB camera intrinsics
        points_2d_rgb = self._project_points_to_image_with_K(points_rgb, self.K_rgb)
        
        # Use vectorized operations for efficiency
        u_coords = points_2d_rgb[:, 0].astype(np.int32)
        v_coords = points_2d_rgb[:, 1].astype(np.int32)
        
        # Check bounds in RGB image
        valid_bounds = (u_coords >= 0) & (u_coords < W) & \
                       (v_coords >= 0) & (v_coords < H)
        
        # Check mask membership (only for points within bounds)
        in_mask = np.zeros(N, dtype=bool)
        valid_indices = np.where(valid_bounds)[0]
        
        if len(valid_indices) > 0:
            in_mask[valid_indices] = mask[v_coords[valid_indices], u_coords[valid_indices]]
        
        # Optional: Apply dilation to include nearby points
        dilation_kernel_size = getattr(self.args, 'mask_dilation', 0)
        if dilation_kernel_size > 0 and np.sum(in_mask) > 0:
            # Dilate mask and re-check
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                               (dilation_kernel_size, dilation_kernel_size))
            mask_dilated = cv2.dilate(mask.astype(np.uint8), kernel, iterations=1) > 0
            in_mask[valid_indices] = mask_dilated[v_coords[valid_indices], u_coords[valid_indices]]
        
        # Additional depth filtering: remove points that are too far or too close
        z_values = pointcloud[:, 2]  # Use original IR frame z values
        valid_depth = (z_values > 0.01) & (z_values < self.args.z_far)
        
        # Combine all filters
        final_mask = in_mask & valid_depth
        
        num_selected = np.sum(final_mask)
        logging.info(f"Mask projection: {np.sum(mask)} mask pixels, "
                    f"{np.sum(valid_bounds)} points in bounds, "
                    f"{num_selected} points selected")
        
        if num_selected == 0:
            logging.warning("No points found within mask! Check IR-RGB calibration.")
            # Debug: show distribution of projected points
            if len(valid_indices) > 0:
                u_range = (u_coords[valid_bounds].min(), u_coords[valid_bounds].max())
                v_range = (v_coords[valid_bounds].min(), v_coords[valid_bounds].max())
                logging.warning(f"Projected points U range: {u_range}, V range: {v_range}")
                logging.warning(f"Mask non-zero region: rows {np.where(mask.any(axis=1))[0][[0,-1]]}, "
                              f"cols {np.where(mask.any(axis=0))[0][[0,-1]]}")
            return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)
        
        # Return points in original IR frame (for consistency with rest of pipeline)
        object_points = pointcloud[final_mask]
        object_colors = colors[final_mask] if len(colors) == len(pointcloud) else \
                       np.ones((num_selected, 3), dtype=np.uint8) * 128
        
        return object_points, object_colors
    
    def _project_points_to_image_with_K(self, points: np.ndarray, K: np.ndarray) -> np.ndarray:
        """Project 3D points to 2D image coordinates using specified intrinsics."""
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        # Avoid division by zero
        z = np.clip(z, 0.001, None)
        
        u = fx * x / z + cx
        v = fy * y / z + cy
        
        return np.stack([u, v], axis=1)
    
    def _project_points_to_image(self, points: np.ndarray) -> np.ndarray:
        """Project 3D points to 2D image coordinates."""
        # points: (N, 3)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        # Avoid division by zero
        z = np.clip(z, 0.001, None)
        
        u = fx * x / z + cx
        v = fy * y / z + cy
        
        return np.stack([u, v], axis=1)
    
    def preprocess_points(self, points: np.ndarray, target_num_points: int = 1024):
        """
        Preprocess point cloud for GraspGen input.
        
        Args:
            points: Object points (N, 3)
            target_num_points: Target number of points for model input
            
        Returns:
            processed_points: Preprocessed points (target_num_points, 3)
        """
        if len(points) == 0:
            return None
        
        # Remove outliers
        points_tensor = torch.from_numpy(points).float()
        filtered_points, removed_points = point_cloud_outlier_removal(points_tensor)
        points = filtered_points.numpy()
        
        if len(points) < 50:
            logging.warning(f"Too few points after outlier removal: {len(points)}")
            return None
        
        logging.debug(f"After outlier removal: {len(points)} points "
                    f"(removed {len(removed_points)})")
        
        # Downsample or upsample to target number
        if len(points) > target_num_points:
            # Random downsample
            indices = np.random.choice(len(points), target_num_points, replace=False)
            points = points[indices]
        elif len(points) < target_num_points:
            # Upsample by repeating points
            repeat_factor = int(np.ceil(target_num_points / len(points)))
            points = np.tile(points, (repeat_factor, 1))[:target_num_points]
        
        return points.astype(np.float32)
    
    def run_inference(self, object_points: np.ndarray):
        """
        Run GraspGen inference on object point cloud.
        
        Args:
            object_points: Object point cloud (N, 3)
            
        Returns:
            grasps: Grasp poses (M, 4, 4) as homogeneous transforms
            confidences: Grasp confidence scores (M,)
        """
        start_time = time.time()
        
        grasps, confidences = GraspGenSampler.run_inference(
            object_points,
            self.grasp_sampler,
            grasp_threshold=self.args.grasp_threshold,
            num_grasps=self.args.num_grasps,
            topk_num_grasps=self.args.topk_num_grasps
        )
        
        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)
        
        if len(grasps) == 0:
            logging.warning("No grasps generated!")
            return np.array([]).reshape(0, 4, 4), np.array([])
        
        grasps = grasps.cpu().numpy()
        confidences = confidences.cpu().numpy()
        
        # Ensure homogeneous coordinate
        grasps[:, 3, 3] = 1
        
        logging.debug(f"Generated {len(grasps)} grasps in {inference_time*1000:.1f}ms")
        return grasps, confidences
    
    def visualize_in_meshcat(self, object_points: np.ndarray, object_colors: np.ndarray,
                            grasps: np.ndarray, confidences: np.ndarray):
        """Visualize point cloud and grasps in meshcat."""
        if self.vis is None:
            return
        
        try:
            # Clear previous visualization
            self.vis.delete()
            
            # Center point cloud for visualization
            pc_center = object_points.mean(axis=0)
            pc_centered = object_points - pc_center
            
            # Shift grasps accordingly
            grasps_centered = grasps.copy()
            grasps_centered[:, :3, 3] -= pc_center
            
            # Visualize object point cloud
            visualize_pointcloud(self.vis, "object_pc", pc_centered, 
                               object_colors, size=0.003)
            
            # Visualize coordinate frame at origin
            make_frame(self.vis, "origin", h=0.1, radius=0.003)
            
            # Get colors based on confidence scores
            colors = get_color_from_score(confidences, use_255_scale=True)
            
            # 先用默认颜色绘制所有夹爪
            for i, (grasp, conf, color) in enumerate(zip(grasps_centered, confidences, colors)):
                visualize_grasp(
                    self.vis,
                    f"grasps/{i:03d}",
                    grasp,
                    color=color.tolist(),
                    gripper_name=self.gripper_name,
                    linewidth=0.6
                )
            
            logging.debug(f"Visualized {len(grasps)} grasps in meshcat")
            
            # 保存可视化数据，用于后续高亮更新
            # 注意：必须在绘制完成后再保存，确保数据与可视化一致
            self.vis_pc_center = pc_center
            self.vis_grasps_centered = grasps_centered
            self.vis_confidences = confidences
            
            # 检查是否有待处理的高亮更新
            highlight_idx = -1
            with self.highlight_lock:
                if self.highlight_pending:
                    self.highlight_pending = False
                    highlight_idx = self.highlight_index
            
            # 如果有待处理的高亮，立即应用
            if highlight_idx >= 0 and highlight_idx < len(grasps_centered):
                visualize_grasp(
                    self.vis,
                    f"grasps/{highlight_idx:03d}",
                    grasps_centered[highlight_idx],
                    color=[255, 0, 0],  # 红色
                    gripper_name=self.gripper_name,
                    linewidth=1.2
                )
                logging.debug(f"Applied pending highlight for grasp #{highlight_idx}")
            
        except Exception as e:
            logging.error(f"Meshcat visualization error: {e}")
    
    def create_pose_array_msg(self, grasps: np.ndarray, frame_id: str = 'camera_link') -> 'PoseArray':
        """
        Create PoseArray message from grasp poses.
        
        Args:
            grasps: Grasp poses (M, 4, 4)
            frame_id: TF frame ID
            
        Returns:
            PoseArray message
        """
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        for grasp in grasps:
            pose = Pose()
            
            # Extract translation
            pose.position.x = float(grasp[0, 3])
            pose.position.y = float(grasp[1, 3])
            pose.position.z = float(grasp[2, 3])
            
            # Extract rotation as quaternion
            rot = R.from_matrix(grasp[:3, :3])
            quat = rot.as_quat()  # [x, y, z, w]
            
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            
            msg.poses.append(pose)
        
        return msg
    
    def run(self):
        """Main run loop."""
        if ROS_AVAILABLE:
            self._run_ros_mode()
        else:
            self._run_standalone_mode()
    
    def _run_ros_mode(self):
        """Run in ROS mode with topic subscription/publishing."""
        logging.info("Running in ROS mode.")
        logging.info("")
        
        # Create spinner thread (needed for camera_info callbacks)
        spin_thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        spin_thread.start()
        
        # Wait for camera parameters from ROS
        logging.info("Fetching camera parameters from ROS...")
        if self.cam_manager.wait_for_camera_info(timeout=10.0):
            self._update_camera_params()
            logging.info(f"Camera params: IR {self.ir_resolution[1]}x{self.ir_resolution[0]}, "
                        f"RGB {self.rgb_resolution[1]}x{self.rgb_resolution[0]}")
        else:
            logging.warning("Using default camera parameters")
            self.camera_info_received = True  # Use defaults
        
        logging.info("")
        logging.info("Waiting for point cloud and mask...")
        logging.info("Required topics:")
        logging.info("  - /grasp_6d/pointcloud (from perception_stereo.py)")
        logging.info("  - /grasp_6d/mask (from segmentation_sam2.py)")
        logging.info("")
        
        last_status_time = 0
        
        try:
            while rclpy.ok():
                # Check for new data
                with self.data_lock:
                    has_pc = self.pointcloud is not None
                    has_mask = self.mask is not None
                    has_new = self.new_data_available
                    
                    if not has_new or not has_pc or not has_mask:
                        # Print status every 5 seconds
                        if time.time() - last_status_time > 5.0:
                            logging.info(f"Waiting... pointcloud: {'YES' if has_pc else 'NO'}, "
                                       f"mask: {'YES' if has_mask else 'NO'}")
                            last_status_time = time.time()
                        time.sleep(0.1)
                        continue
                    
                    pointcloud = self.pointcloud.copy()
                    colors = self.pointcloud_colors.copy() if self.pointcloud_colors is not None else None
                    mask = self.mask.copy()
                    depth = self.depth_image.copy() if self.depth_image is not None else None
                    self.new_data_available = False
                
                # Extract object points from mask
                object_points, object_colors = self.extract_object_points(
                    pointcloud, colors, mask, depth)
                
                if len(object_points) == 0:
                    logging.warning("No object points extracted, skipping...")
                    continue
                
                # Preprocess for GraspGen
                processed_points = self.preprocess_points(object_points, 
                                                         target_num_points=1024)
                if processed_points is None:
                    continue
                
                # Run GraspGen inference
                grasps, confidences = self.run_inference(processed_points)
                
                if len(grasps) == 0:
                    continue
                
                # Store results
                with self.grasp_lock:
                    self.grasp_poses = grasps
                    self.grasp_confidences = confidences
                
                # Publish grasp poses
                pose_msg = self.create_pose_array_msg(grasps, 'camera_link')
                self.grasp_pub.publish(pose_msg)
                
                # Publish confidences
                conf_msg = Float32MultiArray()
                conf_msg.data = confidences.tolist()
                self.conf_pub.publish(conf_msg)
                
                # Visualize in meshcat
                self.visualize_in_meshcat(object_points, object_colors, grasps, confidences)
                
                logging.info(f"Published {len(grasps)} grasp poses")
                logging.info("Press Enter to regenerate grasps, or Ctrl+C to quit...")
                
                # Wait for user to press Enter before regenerating
                try:
                    input()
                    logging.info("Regenerating grasps...")
                except EOFError:
                    # Non-interactive mode, continue
                    pass
                
        except KeyboardInterrupt:
            logging.info("Shutting down...")
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
    
    def _run_standalone_mode(self):
        """Run in standalone mode with sample data."""
        logging.info("Running in standalone mode (ROS not available)")
        
        # Look for sample data
        sample_data_dir = os.path.join(GRASPGEN_DIR, 'GraspGenModels', 'sample_data', 'real_object_pc')
        
        if not os.path.exists(sample_data_dir):
            logging.error(f"Sample data directory not found: {sample_data_dir}")
            return
        
        import glob
        json_files = glob.glob(os.path.join(sample_data_dir, '*.json'))
        
        if not json_files:
            logging.error("No sample JSON files found")
            return
        
        for json_file in json_files:
            logging.info(f"\nProcessing: {os.path.basename(json_file)}")
            
            # Load sample data
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            pc = np.array(data['pc'], dtype=np.float32)
            pc_color = np.array(data.get('pc_color', [[128, 128, 128]] * len(pc)), dtype=np.uint8)
            
            logging.info(f"Loaded {len(pc)} points")
            
            # Preprocess
            processed_points = self.preprocess_points(pc, target_num_points=1024)
            if processed_points is None:
                continue
            
            # Run inference
            grasps, confidences = self.run_inference(processed_points)
            
            if len(grasps) == 0:
                continue
            
            # Visualize
            self.visualize_in_meshcat(pc, pc_color, grasps, confidences)
            
            input("Press Enter to continue to next object...")
        
        logging.info("Standalone mode completed")


def main():
    parser = argparse.ArgumentParser(description='GraspGen Grasp Generation Node')
    parser.add_argument('--gripper_config', type=str,
                       default=os.path.join(GRASPGEN_DIR, 'GraspGenModels/checkpoints/graspgen_franka_panda.yml'),
                       help='Path to gripper configuration YAML')
    parser.add_argument('--grasp_threshold', type=float, default=0.8,
                       help='Minimum confidence threshold for grasps')
    parser.add_argument('--num_grasps', type=int, default=200,
                       help='Number of grasps to generate')
    parser.add_argument('--topk_num_grasps', type=int, default=100,
                       help='Number of top grasps to return')
    parser.add_argument('--z_far', type=float, default=3.0,
                       help='Maximum depth to consider (meters)')
    parser.add_argument('--mask_dilation', type=int, default=0,
                       help='Dilation kernel size for mask (0 to disable). '
                            'Use small values (3-5) if some edge points are missed.')
    
    args = parser.parse_args()
    
    # Set up logging
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    
    logging.info("=" * 60)
    logging.info("GraspGen Grasp Generation Node")
    logging.info("=" * 60)
    logging.info(f"Gripper config: {args.gripper_config}")
    logging.info(f"Grasp threshold: {args.grasp_threshold}")
    logging.info(f"Num grasps: {args.num_grasps}, Top-K: {args.topk_num_grasps}")
    logging.info("=" * 60)
    
    node = GraspGenerationNode(args)
    node.run()


if __name__ == '__main__':
    main()
