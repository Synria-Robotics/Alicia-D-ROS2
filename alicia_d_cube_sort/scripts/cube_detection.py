#!/usr/bin/env python3
"""
Cube Detection Node for Alicia D Robot

This node detects colored cubes (green/blue) using HSV color space and
estimates their 3D positions using either:
1. A4 paper plane reference method (for users without depth camera)
2. Depth camera direct measurement

Features:
- Subscribes to camera image and camera info topics
- Loads and prints camera intrinsic/extrinsic parameters
- Uses CLAHE for brightness normalization
- Publishes detected cube positions as PoseArray messages
- Supports visualization with OpenCV window

Usage:
1. Start the robot and camera:
   ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm
   ros2 launch orbbec_camera gemini_335.launch.py

2. Start cube detection:
   ros2 launch alicia_d_cube_sort cube_detection.launch.py

Author: Synria Robotics
Date: 2026-01
"""

import os
import yaml
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ament_index_python.packages import get_package_share_directory


class CubeDetector(Node):
    """
    Cube Detection Node
    
    Detects colored cubes using HSV color space and estimates 3D positions.
    """
    
    def __init__(self):
        super().__init__('cube_detector')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._get_parameters()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be set from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Camera extrinsics (hand-eye calibration)
        self.T_gripper_camera = None
        self.extrinsics_loaded = False
        
        # Depth image (for depth mode)
        self.depth_image = None
        self.depth_info_received = False
        
        # A4 paper plane detection
        self.a4_plane = None  # (normal, point, corners)
        
        # QoS profile for image topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers for detected cube positions
        self.pub_cubes_green = self.create_publisher(
            PoseArray, '/vision/cubes/green', 10)
        self.pub_cubes_blue = self.create_publisher(
            PoseArray, '/vision/cubes/blue', 10)
        self.pub_detection_image = self.create_publisher(
            Image, '/vision/detection_image', 10)
        
        # Subscribers
        self.sub_color_image = self.create_subscription(
            Image, self.color_topic, self._color_image_callback, qos_profile)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, self.color_info_topic, self._camera_info_callback, 10)
        
        if self.depth_mode:
            self.sub_depth_image = self.create_subscription(
                Image, self.depth_topic, self._depth_image_callback, qos_profile)
            self.sub_depth_info = self.create_subscription(
                CameraInfo, self.depth_info_topic, self._depth_info_callback, 10)
        
        # Load calibration data
        self._load_calibration()
        
        # TF2 for coordinate transformation (camera -> base)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = False
        
        # Print startup info
        self._print_startup_info()
    
    def _declare_parameters(self):
        """Declare ROS parameters"""
        self.declare_parameter('config_file', '')
        self.declare_parameter('depth_mode', False)
        self.declare_parameter('show_image', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        
        # Camera topics
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        
        # Calibration file
        self.declare_parameter('calibration_file', 'hand_eye_calibration_result.yaml')
    
    def _get_parameters(self):
        """Get parameter values"""
        self.config_file = self.get_parameter('config_file').value
        self.depth_mode = self.get_parameter('depth_mode').value
        self.show_image = self.get_parameter('show_image').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        self.color_topic = self.get_parameter('color_topic').value
        self.color_info_topic = self.get_parameter('color_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_info_topic = self.get_parameter('depth_info_topic').value
        
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # Load config file if provided
        self.config = self._load_config()
        
        # Color detection parameters
        self.color_ranges = self.config.get('color_ranges', {
            'green': {'lower': [35, 80, 80], 'upper': [85, 255, 255]},
            'blue': {'lower': [90, 80, 80], 'upper': [130, 255, 255]}
        })
        
        detection = self.config.get('detection', {})
        self.min_area = detection.get('min_area', 150)
        self.max_area = detection.get('max_area', 10000)
        self.min_solidity = detection.get('min_solidity', 0.8)
        self.aspect_ratio_min = detection.get('aspect_ratio_min', 0.7)
        self.aspect_ratio_max = detection.get('aspect_ratio_max', 1.4)
        self.clahe_clip_limit = detection.get('clahe_clip_limit', 2.0)
        self.clahe_tile_size = tuple(detection.get('clahe_tile_size', [8, 8]))
        
        # A4 paper parameters
        a4 = self.config.get('a4_paper', {})
        self.a4_width = a4.get('width', 0.297)
        self.a4_height = a4.get('height', 0.210)
        
        # Cube size
        cube = self.config.get('cube', {})
        self.cube_size = cube.get('size', 0.020)
    
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
    
    def _load_calibration(self):
        """Load hand-eye calibration data"""
        calibration_path = self.calibration_file
        
        # Try to find the calibration file
        if not os.path.isabs(calibration_path):
            try:
                # Look in alicia_d_calibration package
                pkg_share = get_package_share_directory('alicia_d_calibration')
                workspace_root = os.path.abspath(
                    os.path.join(pkg_share, '..', '..', '..', '..'))
                calibration_path = os.path.join(
                    workspace_root, 'src', 'alicia_d_calibration', 'config', 
                    self.calibration_file)
            except Exception:
                pass
        
        if not os.path.exists(calibration_path):
            self.get_logger().error(f'Calibration file not found: {calibration_path}')
            return
        
        try:
            with open(calibration_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            hand_eye = calib_data.get('hand_eye_calibration', {})
            transform = hand_eye.get('transform', {})
            
            # Get translation
            trans = transform.get('translation', {})
            t = np.array([trans.get('x', 0), trans.get('y', 0), trans.get('z', 0)])
            
            # Get rotation (quaternion)
            rot = transform.get('rotation', {}).get('quaternion', {})
            q = [rot.get('x', 0), rot.get('y', 0), rot.get('z', 0), rot.get('w', 1)]
            
            # Build transformation matrix
            r = R.from_quat(q)
            self.T_gripper_camera = np.eye(4)
            self.T_gripper_camera[:3, :3] = r.as_matrix()
            self.T_gripper_camera[:3, 3] = t
            
            self.extrinsics_loaded = True
            self.get_logger().info(f'Loaded calibration from: {calibration_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')
    
    def _print_startup_info(self):
        """Print startup information including camera parameters"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Cube Detection Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'3D Mode: {"Depth Camera" if self.depth_mode else "A4 Paper Plane"}')
        self.get_logger().info(f'Color Topic: {self.color_topic}')
        self.get_logger().info(f'Camera Info Topic: {self.color_info_topic}')
        if self.depth_mode:
            self.get_logger().info(f'Depth Topic: {self.depth_topic}')
        self.get_logger().info(f'Show Image: {self.show_image}')
        self.get_logger().info('-' * 60)
        
        # Print color ranges
        self.get_logger().info('Color Detection Ranges (HSV):')
        for color, ranges in self.color_ranges.items():
            self.get_logger().info(
                f'  {color}: [{ranges["lower"]}] - [{ranges["upper"]}]')
        
        # Print calibration info
        if self.extrinsics_loaded:
            self.get_logger().info('-' * 60)
            self.get_logger().info('Hand-Eye Calibration (Extrinsics):')
            self.get_logger().info(f'  T_gripper_camera translation: {self.T_gripper_camera[:3, 3]}')
            euler = R.from_matrix(self.T_gripper_camera[:3, :3]).as_euler('xyz', degrees=True)
            self.get_logger().info(f'  T_gripper_camera rotation (deg): {euler}')
        else:
            self.get_logger().warn('Hand-eye calibration NOT loaded!')
        
        self.get_logger().info('=' * 60)
    
    def _camera_info_callback(self, msg: CameraInfo):
        """Process camera info message"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            
            # Print camera intrinsics
            self.get_logger().info('=' * 60)
            self.get_logger().info('Camera Intrinsics Received:')
            self.get_logger().info(f'  fx: {self.camera_matrix[0, 0]:.2f}')
            self.get_logger().info(f'  fy: {self.camera_matrix[1, 1]:.2f}')
            self.get_logger().info(f'  cx: {self.camera_matrix[0, 2]:.2f}')
            self.get_logger().info(f'  cy: {self.camera_matrix[1, 2]:.2f}')
            self.get_logger().info(f'  Distortion: {self.dist_coeffs}')
            self.get_logger().info('=' * 60)
    
    def _depth_info_callback(self, msg: CameraInfo):
        """Process depth camera info"""
        if not self.depth_info_received:
            self.depth_info_received = True
            self.get_logger().info('Depth camera info received')
    
    def _depth_image_callback(self, msg: Image):
        """Process depth image"""
        try:
            # Convert to numpy array (16UC1 format, values in mm)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth image conversion error: {e}')
    
    def _color_image_callback(self, msg: Image):
        """Process color image and detect cubes"""
        if not self.camera_info_received:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect A4 paper for plane estimation (if not using depth mode)
            if not self.depth_mode:
                self._detect_a4_plane(cv_image)
            
            # Detect colored cubes
            detections = self._detect_cubes(cv_image)
            
            # Estimate 3D positions and publish
            self._process_detections(detections, cv_image, msg.header)
            
            # Show visualization
            if self.show_image:
                cv2.imshow('Cube Detection', cv_image)
                cv2.waitKey(1)
            
            # Publish detection image
            try:
                detection_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                detection_msg.header = msg.header
                self.pub_detection_image.publish(detection_msg)
            except Exception:
                pass
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
    
    def _detect_a4_plane(self, image):
        """Detect A4 paper and estimate plane"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 75, 200)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return
        
        # Sort by area (largest first)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            
            if len(approx) == 4:
                # Found quadrilateral - assume it's A4 paper
                corners = approx.reshape(-1, 2).astype(np.float32)
                corners = self._order_points_clockwise(corners)
                
                # Estimate plane from corners
                plane = self._estimate_plane_from_rectangle(
                    corners, self.a4_width, self.a4_height)
                
                if plane is not None:
                    self.a4_plane = plane
                    # Draw A4 outline
                    cv2.polylines(image, [corners.astype(int)], True, (0, 255, 255), 2)
                    for pt in corners:
                        cv2.circle(image, tuple(pt.astype(int)), 5, (0, 0, 255), -1)
                break
    
    def _order_points_clockwise(self, pts):
        """Order points in clockwise order: TL, TR, BR, BL"""
        center = pts.mean(axis=0)
        angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
        order = np.argsort(angles)
        ordered = pts[order]
        
        # Find top-left (smallest x + y sum)
        sums = ordered.sum(axis=1)
        tl_idx = np.argmin(sums)
        
        # Rotate to start from top-left
        ordered = np.roll(ordered, -tl_idx, axis=0)
        
        return ordered
    
    def _estimate_plane_from_rectangle(self, corners, width, height):
        """Estimate plane from rectangle corners using solvePnP"""
        if self.camera_matrix is None:
            return None
        
        # Determine orientation based on pixel dimensions
        w_px = np.linalg.norm(corners[1] - corners[0])
        h_px = np.linalg.norm(corners[2] - corners[1])
        
        if h_px > w_px:
            width, height = height, width
        
        # 3D object points (rectangle on Z=0 plane)
        obj_points = np.array([
            [0, 0, 0],
            [width, 0, 0],
            [width, height, 0],
            [0, height, 0]
        ], dtype=np.float32)
        
        img_points = corners.reshape(-1, 1, 2)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)
        
        if not success:
            return None
        
        # Convert to rotation matrix
        R_mat, _ = cv2.Rodrigues(rvec)
        
        # Plane normal (Z-axis of the plane in camera frame)
        normal = R_mat[:, 2]
        
        # A point on the plane (center)
        center_3d = R_mat @ np.array([width/2, height/2, 0]) + tvec.flatten()
        
        return (normal, center_3d, corners)
    
    def _pixel_to_plane_point(self, u, v, normal, plane_point):
        """Convert pixel coordinates to 3D point on plane"""
        if self.camera_matrix is None:
            return None
        
        # Camera ray direction
        K_inv = np.linalg.inv(self.camera_matrix)
        ray = K_inv @ np.array([u, v, 1.0])
        ray = ray / np.linalg.norm(ray)
        
        # Plane equation: n · (P - P0) = 0
        # Ray: P = t * ray
        # Solve: n · (t * ray - P0) = 0 => t = (n · P0) / (n · ray)
        n_dot_ray = np.dot(normal, ray)
        if abs(n_dot_ray) < 1e-6:
            return None
        
        t = np.dot(normal, plane_point) / n_dot_ray
        if t < 0:
            return None
        
        return t * ray
    
    def _detect_cubes(self, image):
        """Detect colored cubes in image"""
        detections = {'green': [], 'blue': []}
        
        # Get image brightness for adaptive thresholds
        brightness = np.mean(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        
        # Apply gamma correction for low light
        if brightness < 127.5:
            gamma = 1.7 if brightness >= 76.5 else 2.0
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 
                             for i in np.arange(256)]).astype('uint8')
            image_processed = cv2.LUT(image, table)
        else:
            image_processed = image.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(image_processed, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        
        # Apply CLAHE
        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_limit, 
            tileGridSize=self.clahe_tile_size)
        v = clahe.apply(v)
        s = clahe.apply(s)
        hsv = cv2.merge([h, s, v])
        
        # Detect each color
        for color_name, ranges in self.color_ranges.items():
            lower = np.array(ranges['lower'], dtype=np.uint8)
            upper = np.array(ranges['upper'], dtype=np.uint8)
            
            # Create mask
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.medianBlur(mask, 5)
            mask = cv2.GaussianBlur(mask, (3, 3), 0)
            
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area or area > self.max_area:
                    continue
                
                # Check solidity
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area <= 0:
                    continue
                solidity = area / hull_area
                if solidity < self.min_solidity:
                    continue
                
                # Get bounding box
                rect = cv2.minAreaRect(hull)
                (cx, cy), (w, h), angle = rect
                
                # Check aspect ratio
                if h > 0:
                    aspect = w / h if w > h else h / w
                    if aspect < self.aspect_ratio_min or aspect > self.aspect_ratio_max:
                        continue
                
                # Valid detection
                box = cv2.boxPoints(rect).astype(int)
                cv2.polylines(image, [box], True, (0, 255, 0), 2)
                cv2.circle(image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                cv2.putText(image, color_name, (box[0][0], box[0][1] - 6),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                detections[color_name].append({
                    'center_px': (float(cx), float(cy)),
                    'box': box,
                    'area': area
                })
        
        return detections
    
    def _get_3d_position(self, cx, cy):
        """Get 3D position from pixel coordinates"""
        if self.depth_mode:
            return self._get_3d_from_depth(cx, cy)
        else:
            return self._get_3d_from_a4_plane(cx, cy)
    
    def _get_3d_from_depth(self, cx, cy):
        """Get 3D position using depth camera"""
        if self.depth_image is None:
            return None
        
        # Get depth value at pixel
        x, y = int(cx), int(cy)
        if x < 0 or y < 0 or x >= self.depth_image.shape[1] or y >= self.depth_image.shape[0]:
            return None
        
        # Get depth (typically in mm, convert to m)
        depth = self.depth_image[y, x]
        if depth <= 0:
            return None
        
        depth_m = depth / 1000.0  # mm to m
        
        # Back-project to 3D
        if self.camera_matrix is None:
            return None
        
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx_cam = self.camera_matrix[0, 2]
        cy_cam = self.camera_matrix[1, 2]
        
        x_3d = (cx - cx_cam) * depth_m / fx
        y_3d = (cy - cy_cam) * depth_m / fy
        z_3d = depth_m
        
        return np.array([x_3d, y_3d, z_3d])
    
    def _get_3d_from_a4_plane(self, cx, cy):
        """Get 3D position using A4 paper plane reference"""
        if self.a4_plane is None:
            return None
        
        normal, plane_point, _ = self.a4_plane
        return self._pixel_to_plane_point(cx, cy, normal, plane_point)
    
    def _transform_to_base(self, pos_camera):
        """
        Transform position from camera optical frame to base frame.
        
        Args:
            pos_camera: 3D position in camera optical frame (numpy array)
            
        Returns:
            3D position in base frame (numpy array) or None if transform failed
        """
        try:
            # Get transform from camera to base
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
            
            if not self.tf_ready:
                self.tf_ready = True
                self.get_logger().info(f'TF transform ready: {self.camera_frame} -> {self.base_frame}')
            
            # Extract translation and rotation
            t = transform.transform.translation
            r = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            rot = R.from_quat([r.x, r.y, r.z, r.w])
            
            # Transform point: P_base = R * P_camera + T
            p_camera = np.array(pos_camera)
            p_base = rot.apply(p_camera) + np.array([t.x, t.y, t.z])
            
            return p_base
            
        except TransformException as e:
            if self.tf_ready:
                self.get_logger().warn(f'TF transform failed: {e}')
            return None
    
    def _process_detections(self, detections, image, header):
        """Process detections, transform to base frame, and publish pose arrays"""
        # Pose arrays in BASE frame (not camera frame)
        poses_green = PoseArray()
        poses_green.header = Header()
        poses_green.header.stamp = header.stamp
        poses_green.header.frame_id = self.base_frame  # Changed to base_frame
        
        poses_blue = PoseArray()
        poses_blue.header = Header()
        poses_blue.header.stamp = header.stamp
        poses_blue.header.frame_id = self.base_frame  # Changed to base_frame
        
        # Check TF availability once
        tf_available = self.tf_ready or self._check_tf_available()
        
        for color_name, dets in detections.items():
            pose_array = poses_green if color_name == 'green' else poses_blue
            
            for det in dets:
                cx, cy = det['center_px']
                pos_camera = self._get_3d_position(cx, cy)
                
                if pos_camera is not None:
                    # Transform from camera frame to base frame
                    pos_base = self._transform_to_base(pos_camera)
                    
                    if pos_base is not None:
                        pose = Pose()
                        pose.position.x = float(pos_base[0])
                        pose.position.y = float(pos_base[1])
                        pose.position.z = float(pos_base[2])
                        # Default orientation (facing down for grasping)
                        pose.orientation.w = 1.0
                        pose.orientation.x = 0.0
                        pose.orientation.y = 0.0
                        pose.orientation.z = 0.0
                        pose_array.poses.append(pose)
                        
                        # Draw BASE frame 3D coordinates on image (yellow text)
                        cv2.putText(image, 
                                   f'base: {pos_base[0]:.3f},{pos_base[1]:.3f},{pos_base[2]:.3f}',
                                   (int(cx) + 10, int(cy) + 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    else:
                        # TF not available, show camera frame coordinates
                        cv2.putText(image, 
                                   f'cam: {pos_camera[0]:.3f},{pos_camera[1]:.3f},{pos_camera[2]:.3f}',
                                   (int(cx) + 10, int(cy) + 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 255), 1)
        
        # Display mode and TF status on image
        mode_text = f"MODE: {'DEPTH' if self.depth_mode else 'A4_PLANE'}"
        cv2.putText(image, mode_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Display TF status
        tf_status = "TF: OK" if self.tf_ready else "TF: WAITING"
        tf_color = (0, 255, 0) if self.tf_ready else (0, 0, 255)
        cv2.putText(image, tf_status, (image.shape[1] - 120, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, tf_color, 2)
        
        # Display detection counts
        green_count = len(poses_green.poses)
        blue_count = len(poses_blue.poses)
        cv2.putText(image, f"Green: {green_count}, Blue: {blue_count}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display coordinate frame info
        cv2.putText(image, f"Frame: {self.base_frame}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Publish (poses are in base_frame)
        self.pub_cubes_green.publish(poses_green)
        self.pub_cubes_blue.publish(poses_blue)
    
    def _check_tf_available(self):
        """Check if TF transform is available"""
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
            self.tf_ready = True
            return True
        except TransformException:
            return False
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.show_image:
            cv2.destroyAllWindows()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = CubeDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
