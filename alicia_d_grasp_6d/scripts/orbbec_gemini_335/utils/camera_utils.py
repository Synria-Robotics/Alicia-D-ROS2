"""
Camera parameter utilities for Orbbec Gemini 335.

This module provides functions for dynamically obtaining camera parameters from ROS:
- Camera intrinsics from CameraInfo topics
- Extrinsic transforms from TF2

Usage:
    # With ROS node
    from utils.camera_utils import CameraParameterManager
    
    cam_manager = CameraParameterManager(node)
    cam_manager.wait_for_camera_info()
    
    K_ir = cam_manager.K_ir
    K_rgb = cam_manager.K_rgb
    T_ir_to_rgb = cam_manager.T_ir_to_rgb
"""

import numpy as np
import threading
import time
import logging
from typing import Optional, Tuple, Dict, Any

# ROS 2 imports (optional - for non-ROS environments)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo
    from tf2_ros import Buffer, TransformListener, TransformException
    from geometry_msgs.msg import TransformStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# Default camera intrinsics for Orbbec Gemini 335
# These are fallback values if ROS topics are not available
DEFAULT_IR_K = np.array([
    [411.666748046875, 0.0, 420.0250244140625],
    [0.0, 411.666748046875, 240.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DEFAULT_RGB_K = np.array([
    [693.5951538085938, 0.0, 648.8980102539062],
    [0.0, 693.14501953125, 361.5228271484375],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DEFAULT_IR_RESOLUTION = (480, 848)  # (H, W)
DEFAULT_RGB_RESOLUTION = (720, 1280)  # (H, W)

# Default IR to RGB extrinsic transform
# From: ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_left_ir_optical_frame
DEFAULT_T_IR_TO_RGB = np.array([
    [1.000, -0.000, -0.002, -0.014],
    [0.001,  1.000,  0.004, -0.000],
    [0.002, -0.004,  1.000, -0.002],
    [0.000,  0.000,  0.000,  1.000]
], dtype=np.float32)

DEFAULT_BASELINE = 0.05  # 50mm baseline for stereo pair


class CameraParameterManager:
    """
    Manages camera parameters by subscribing to ROS topics and TF2.
    
    Automatically fetches:
    - IR camera intrinsics from /camera/left_ir/camera_info
    - RGB camera intrinsics from /camera/color/camera_info  
    - IR to RGB extrinsic transform from TF2
    
    Falls back to default values if ROS is not available or topics timeout.
    """
    
    # Topic names
    IR_CAMERA_INFO_TOPIC = '/camera/left_ir/camera_info'
    RGB_CAMERA_INFO_TOPIC = '/camera/color/camera_info'
    
    # TF frame names
    IR_OPTICAL_FRAME = 'camera_left_ir_optical_frame'
    RGB_OPTICAL_FRAME = 'camera_color_optical_frame'
    DEPTH_OPTICAL_FRAME = 'camera_depth_optical_frame'
    
    def __init__(self, node: Optional['Node'] = None, use_defaults: bool = False):
        """
        Initialize camera parameter manager.
        
        Args:
            node: ROS 2 node for subscriptions (optional)
            use_defaults: If True, skip ROS and use default values
        """
        self.node = node
        self.use_defaults = use_defaults or not ROS_AVAILABLE
        
        # Camera intrinsics
        self._K_ir = DEFAULT_IR_K.copy()
        self._K_rgb = DEFAULT_RGB_K.copy()
        self._ir_resolution = DEFAULT_IR_RESOLUTION
        self._rgb_resolution = DEFAULT_RGB_RESOLUTION
        self._baseline = DEFAULT_BASELINE
        
        # Extrinsic transform
        self._T_ir_to_rgb = DEFAULT_T_IR_TO_RGB.copy()
        
        # Distortion coefficients (optional)
        self._D_ir = None
        self._D_rgb = None
        
        # Status flags
        self._ir_info_received = False
        self._rgb_info_received = False
        self._tf_received = False
        self._lock = threading.Lock()
        
        # TF2 buffer
        self._tf_buffer = None
        self._tf_listener = None
        
        if not self.use_defaults and self.node is not None:
            self._init_ros_interfaces()
    
    def _init_ros_interfaces(self):
        """Initialize ROS subscribers and TF listener."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Camera info subscribers
        self._ir_info_sub = self.node.create_subscription(
            CameraInfo, self.IR_CAMERA_INFO_TOPIC,
            self._ir_camera_info_callback, sensor_qos)
        
        self._rgb_info_sub = self.node.create_subscription(
            CameraInfo, self.RGB_CAMERA_INFO_TOPIC,
            self._rgb_camera_info_callback, sensor_qos)
        
        # TF2 buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        
        logging.info(f"Subscribed to: {self.IR_CAMERA_INFO_TOPIC}, {self.RGB_CAMERA_INFO_TOPIC}")
    
    def _ir_camera_info_callback(self, msg: 'CameraInfo'):
        """Process IR camera info message."""
        with self._lock:
            self._K_ir = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self._ir_resolution = (msg.height, msg.width)
            if len(msg.d) > 0:
                self._D_ir = np.array(msg.d, dtype=np.float32)
            self._ir_info_received = True
            
        if not getattr(self, '_ir_logged', False):
            logging.info(f"IR camera info received: {msg.width}x{msg.height}, "
                        f"fx={self._K_ir[0,0]:.2f}")
            self._ir_logged = True
    
    def _rgb_camera_info_callback(self, msg: 'CameraInfo'):
        """Process RGB camera info message."""
        with self._lock:
            self._K_rgb = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self._rgb_resolution = (msg.height, msg.width)
            if len(msg.d) > 0:
                self._D_rgb = np.array(msg.d, dtype=np.float32)
            self._rgb_info_received = True
            
        if not getattr(self, '_rgb_logged', False):
            logging.info(f"RGB camera info received: {msg.width}x{msg.height}, "
                        f"fx={self._K_rgb[0,0]:.2f}")
            self._rgb_logged = True
    
    def _fetch_tf_transform(self, timeout: float = 2.0) -> bool:
        """
        Fetch IR to RGB transform from TF2.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if transform was successfully fetched
        """
        if self._tf_buffer is None:
            return False
        
        try:
            # tf2_echo A B gives transform FROM B TO A
            # So we query rgb <- ir to get transform FROM ir TO rgb
            transform = self._tf_buffer.lookup_transform(
                self.RGB_OPTICAL_FRAME,
                self.IR_OPTICAL_FRAME,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )
            
            # Convert to 4x4 matrix
            self._T_ir_to_rgb = self._transform_to_matrix(transform)
            self._tf_received = True
            
            logging.info(f"TF transform received: {self.IR_OPTICAL_FRAME} -> {self.RGB_OPTICAL_FRAME}")
            logging.debug(f"Translation: [{self._T_ir_to_rgb[0,3]:.4f}, "
                         f"{self._T_ir_to_rgb[1,3]:.4f}, {self._T_ir_to_rgb[2,3]:.4f}]")
            return True
            
        except TransformException as e:
            logging.warning(f"Failed to get TF transform: {e}")
            return False
    
    def _transform_to_matrix(self, transform: 'TransformStamped') -> np.ndarray:
        """Convert ROS TransformStamped to 4x4 matrix."""
        from scipy.spatial.transform import Rotation as R
        
        t = transform.transform.translation
        q = transform.transform.rotation
        
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        
        return T
    
    def wait_for_camera_info(self, timeout: float = 10.0) -> bool:
        """
        Wait for camera info to be received.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if all camera info was received
        """
        if self.use_defaults:
            logging.info("Using default camera parameters (ROS not available)")
            return True
        
        start_time = time.time()
        rate = 0.1  # Check every 100ms
        
        logging.info("Waiting for camera info...")
        
        while time.time() - start_time < timeout:
            with self._lock:
                ir_ok = self._ir_info_received
                rgb_ok = self._rgb_info_received
            
            if ir_ok and rgb_ok:
                # Also try to get TF transform
                self._fetch_tf_transform(timeout=2.0)
                logging.info("Camera info received successfully")
                return True
            
            time.sleep(rate)
        
        logging.warning(f"Timeout waiting for camera info "
                       f"(IR: {self._ir_info_received}, RGB: {self._rgb_info_received}). "
                       "Using defaults.")
        return False
    
    @property
    def K_ir(self) -> np.ndarray:
        """IR camera intrinsic matrix (3x3)."""
        with self._lock:
            return self._K_ir.copy()
    
    @property
    def K_rgb(self) -> np.ndarray:
        """RGB camera intrinsic matrix (3x3)."""
        with self._lock:
            return self._K_rgb.copy()
    
    @property
    def ir_resolution(self) -> Tuple[int, int]:
        """IR camera resolution (H, W)."""
        with self._lock:
            return self._ir_resolution
    
    @property
    def rgb_resolution(self) -> Tuple[int, int]:
        """RGB camera resolution (H, W)."""
        with self._lock:
            return self._rgb_resolution
    
    @property
    def T_ir_to_rgb(self) -> np.ndarray:
        """Transform from IR optical frame to RGB optical frame (4x4)."""
        with self._lock:
            return self._T_ir_to_rgb.copy()
    
    @property
    def baseline(self) -> float:
        """Stereo baseline in meters."""
        with self._lock:
            return self._baseline
    
    @property
    def is_ready(self) -> bool:
        """Check if all camera parameters are available."""
        if self.use_defaults:
            return True
        with self._lock:
            return self._ir_info_received and self._rgb_info_received
    
    def get_all_params(self) -> Dict[str, Any]:
        """
        Get all camera parameters as a dictionary.
        
        Returns:
            Dictionary with all camera parameters
        """
        with self._lock:
            return {
                'K_ir': self._K_ir.copy(),
                'K_rgb': self._K_rgb.copy(),
                'ir_resolution': self._ir_resolution,
                'rgb_resolution': self._rgb_resolution,
                'T_ir_to_rgb': self._T_ir_to_rgb.copy(),
                'baseline': self._baseline,
                'D_ir': self._D_ir.copy() if self._D_ir is not None else None,
                'D_rgb': self._D_rgb.copy() if self._D_rgb is not None else None,
            }
    
    def save_to_json(self, filepath: str):
        """
        Save camera parameters to JSON file.
        
        Args:
            filepath: Output JSON file path
        """
        import json
        
        params = self.get_all_params()
        
        # Convert numpy arrays to lists for JSON serialization
        json_params = {}
        for key, value in params.items():
            if isinstance(value, np.ndarray):
                json_params[key] = value.tolist()
            else:
                json_params[key] = value
        
        with open(filepath, 'w') as f:
            json.dump(json_params, f, indent=2)
        
        logging.info(f"Camera parameters saved to {filepath}")
    
    @classmethod
    def load_from_json(cls, filepath: str) -> 'CameraParameterManager':
        """
        Load camera parameters from JSON file.
        
        Args:
            filepath: Input JSON file path
            
        Returns:
            CameraParameterManager with loaded parameters
        """
        import json
        
        with open(filepath, 'r') as f:
            params = json.load(f)
        
        manager = cls(node=None, use_defaults=True)
        
        if 'K_ir' in params:
            manager._K_ir = np.array(params['K_ir'], dtype=np.float32)
        if 'K_rgb' in params:
            manager._K_rgb = np.array(params['K_rgb'], dtype=np.float32)
        if 'ir_resolution' in params:
            manager._ir_resolution = tuple(params['ir_resolution'])
        if 'rgb_resolution' in params:
            manager._rgb_resolution = tuple(params['rgb_resolution'])
        if 'T_ir_to_rgb' in params:
            manager._T_ir_to_rgb = np.array(params['T_ir_to_rgb'], dtype=np.float32)
        if 'baseline' in params:
            manager._baseline = params['baseline']
        
        logging.info(f"Camera parameters loaded from {filepath}")
        return manager


def get_camera_intrinsics_from_topic(topic: str, timeout: float = 5.0) -> Optional[np.ndarray]:
    """
    Standalone function to get camera intrinsics from a single topic.
    
    This creates a temporary node to fetch the camera info.
    
    Args:
        topic: Camera info topic name
        timeout: Timeout in seconds
        
    Returns:
        Camera intrinsic matrix (3x3) or None if timeout
    """
    if not ROS_AVAILABLE:
        logging.warning("ROS not available, cannot fetch camera intrinsics")
        return None
    
    # Initialize ROS if not already
    if not rclpy.ok():
        rclpy.init()
    
    node = rclpy.create_node('_camera_intrinsics_fetcher')
    
    result = {'K': None, 'received': False}
    
    def callback(msg):
        result['K'] = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        result['received'] = True
    
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )
    
    sub = node.create_subscription(CameraInfo, topic, callback, qos)
    
    start = time.time()
    while not result['received'] and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    
    return result['K']


def get_tf_transform(source_frame: str, target_frame: str, 
                     timeout: float = 5.0) -> Optional[np.ndarray]:
    """
    Standalone function to get a TF transform.
    
    Args:
        source_frame: Source TF frame
        target_frame: Target TF frame  
        timeout: Timeout in seconds
        
    Returns:
        4x4 transform matrix or None if timeout
    """
    if not ROS_AVAILABLE:
        logging.warning("ROS not available, cannot fetch TF transform")
        return None
    
    from scipy.spatial.transform import Rotation as R
    
    # Initialize ROS if not already
    if not rclpy.ok():
        rclpy.init()
    
    node = rclpy.create_node('_tf_fetcher')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    # Wait for TF to be ready
    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        
        try:
            transform = tf_buffer.lookup_transform(
                target_frame, source_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5)
            )
            
            # Convert to matrix
            t = transform.transform.translation
            q = transform.transform.rotation
            
            T = np.eye(4, dtype=np.float32)
            T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [t.x, t.y, t.z]
            
            node.destroy_node()
            return T
            
        except TransformException:
            continue
    
    node.destroy_node()
    logging.warning(f"Timeout waiting for TF: {source_frame} -> {target_frame}")
    return None
