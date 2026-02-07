#!/usr/bin/env python3
"""
ROS 2 Bridge for 6D Grasp Pipeline (Intel RealSense D405)

This script runs in system Python (with ROS 2) and bridges camera data
to the perception scripts running in conda environments with different Python versions.

Bridges:
1. Stereo IR images -> FoundationStereo (d405_foundationstereo.py)
2. RGB images -> SAM2 (d405_sam2.py)
3. Receives point clouds and masks, republishes to ROS topics

Usage:
    # In system Python environment (not conda)
    python3 d405_ros_bridge.py [--stereo-port 5555] [--rgb-port 5557]
"""

import os
import sys
import argparse
import time
import threading
import struct
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header, Empty
import message_filters

# ZeroMQ for inter-process communication
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[WARNING] ZeroMQ not available. Install with: pip install pyzmq")
    print("[INFO] Will use file-based communication only.")


class ROSBridge(Node):
    """
    Unified ROS 2 bridge for the 6D grasp pipeline (Intel RealSense D405).
    
    Handles:
    - Stereo IR images for FoundationStereo
    - RGB images for SAM2
    - Point cloud and mask republishing
    """
    
    def __init__(self, args):
        super().__init__('ros_grasp_bridge_d405')
        self.args = args
        
        # Shared directory for file-based communication
        self.bridge_dir = os.path.join(os.path.dirname(__file__), '.bridge_data')
        os.makedirs(self.bridge_dir, exist_ok=True)
        
        # Clear stale point cloud files on startup to force fresh generation
        self._clear_stale_pointcloud_files()
        
        # Data buffers
        self.stereo_left = None
        self.stereo_right = None
        self.rgb_image = None
        self.camera_info = None
        self.data_lock = threading.Lock()
        
        # Timestamps for file-based communication
        self.last_mask_timestamp = 0
        self.last_pc_timestamp = 0
        
        # Flag to block reading stale PC files during regeneration
        self.regenerate_in_progress = False
        
        # ZeroMQ sockets
        self.zmq_context = None
        self.stereo_pub_socket = None  # Publish stereo images
        self.rgb_pub_socket = None     # Publish RGB images
        self.pc_sub_socket = None      # Subscribe to point clouds
        self.mask_sub_socket = None    # Subscribe to masks
        
        if ZMQ_AVAILABLE:
            self._init_zmq()
        
        # Initialize ROS
        self._init_ros()
        
        self.get_logger().info("ROS Bridge (D405) initialized")
        self.get_logger().info(f"Bridge directory: {self.bridge_dir}")
    
    def _clear_stale_pointcloud_files(self):
        """Clear stale point cloud files on startup to force fresh generation.
        
        This ensures that when the system restarts, it doesn't use old cached
        point cloud data from a previous session where objects may have been
        in different positions.
        """
        output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
        files_to_clear = ['pc_timestamp.txt', 'xyz_map.npy', 'colors.npy']
        
        for filename in files_to_clear:
            filepath = os.path.join(output_dir, filename)
            if os.path.exists(filepath):
                try:
                    os.remove(filepath)
                    self.get_logger().info(f"Cleared stale file: {filename}")
                except Exception as e:
                    self.get_logger().warning(f"Failed to clear {filename}: {e}")
        
        self.get_logger().info("Stale point cloud cache cleared, waiting for fresh data from FoundationStereo")
    
    def _init_zmq(self):
        """Initialize ZeroMQ sockets."""
        self.zmq_context = zmq.Context()
        
        # Publisher for stereo images (FoundationStereo subscribes)
        try:
            self.stereo_pub_socket = self.zmq_context.socket(zmq.PUB)
            self.stereo_pub_socket.setsockopt(zmq.SNDHWM, 5)
            self.stereo_pub_socket.bind(f"tcp://*:{self.args.stereo_port}")
            self.get_logger().info(f"Stereo publisher on port {self.args.stereo_port}")
        except zmq.error.ZMQError as e:
            self.get_logger().warning(f"Stereo port {self.args.stereo_port} in use, trying {self.args.stereo_port + 10}")
            try:
                self.stereo_pub_socket.bind(f"tcp://*:{self.args.stereo_port + 10}")
                self.args.stereo_port += 10
                self.get_logger().info(f"Stereo publisher on port {self.args.stereo_port}")
            except:
                self.get_logger().error("Could not bind stereo publisher")
                self.stereo_pub_socket = None
        
        # Publisher for RGB images (SAM2 subscribes)
        try:
            self.rgb_pub_socket = self.zmq_context.socket(zmq.PUB)
            self.rgb_pub_socket.setsockopt(zmq.SNDHWM, 2)
            self.rgb_pub_socket.bind(f"tcp://*:{self.args.rgb_port}")
            self.get_logger().info(f"RGB publisher on port {self.args.rgb_port}")
        except zmq.error.ZMQError as e:
            self.get_logger().warning(f"RGB port {self.args.rgb_port} in use, trying {self.args.rgb_port + 10}")
            try:
                self.rgb_pub_socket.bind(f"tcp://*:{self.args.rgb_port + 10}")
                self.args.rgb_port += 10
                self.get_logger().info(f"RGB publisher on port {self.args.rgb_port}")
            except:
                self.get_logger().error("Could not bind RGB publisher")
                self.rgb_pub_socket = None
        
        # Subscriber for point clouds from FoundationStereo
        try:
            self.pc_sub_socket = self.zmq_context.socket(zmq.SUB)
            self.pc_sub_socket.connect(f"tcp://localhost:{self.args.stereo_port + 1}")
            self.pc_sub_socket.setsockopt_string(zmq.SUBSCRIBE, "pointcloud")
            self.pc_sub_socket.setsockopt(zmq.RCVTIMEO, 50)
            self.get_logger().info(f"Point cloud subscriber on port {self.args.stereo_port + 1}")
        except Exception as e:
            self.get_logger().warning(f"Could not connect PC subscriber: {e}")
            self.pc_sub_socket = None
        
        # Subscriber for masks from SAM2 (SAM2 publishes to rgb_port + 1)
        try:
            self.mask_sub_socket = self.zmq_context.socket(zmq.SUB)
            self.mask_sub_socket.connect(f"tcp://localhost:{self.args.rgb_port + 1}")
            self.mask_sub_socket.setsockopt_string(zmq.SUBSCRIBE, "mask")
            self.mask_sub_socket.setsockopt(zmq.RCVTIMEO, 50)
            self.get_logger().info(f"Mask subscriber on port {self.args.rgb_port + 1}")
        except Exception as e:
            self.get_logger().warning(f"Could not connect mask subscriber: {e}")
            self.mask_sub_socket = None
    
    def _init_ros(self):
        """Initialize ROS subscribers and publishers."""
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
        
        # QoS for latched topics - late subscribers get last message
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # === Stereo IR Subscribers (D405 specific topic names) ===
        self.left_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/infra1/image_rect_raw', qos_profile=sensor_qos)
        self.right_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/infra2/image_rect_raw', qos_profile=sensor_qos)
        
        self.stereo_sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=5, slop=0.05)
        self.stereo_sync.registerCallback(self._stereo_callback)
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/infra1/camera_info',
            self._camera_info_callback, sensor_qos)
        
        # === RGB Subscriber (D405 specific topic name) ===
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_rect_raw',
            self._rgb_callback, sensor_qos)
        
        # === Publishers (for republishing processed data) ===
        # Use latched QoS so late-joining subscribers get the last message
        self.pc_pub = self.create_publisher(
            PointCloud2, '/grasp_6d/pointcloud', latched_qos)
        
        self.mask_pub = self.create_publisher(
            Image, '/grasp_6d/mask', latched_qos)
        
        # Timer to check for processed data
        self.check_timer = self.create_timer(0.05, self._check_processed_data)
        
        # === Regenerate signal (from GraspGen -> FoundationStereo) ===
        self.regenerate_sub = self.create_subscription(
            Empty, '/grasp_6d/regenerate',
            self._regenerate_callback, reliable_qos)
        
        self.get_logger().info("ROS interfaces initialized")
        self.get_logger().info("Subscriptions (D405):")
        self.get_logger().info("  - /camera/camera/infra1/image_rect_raw")
        self.get_logger().info("  - /camera/camera/infra2/image_rect_raw")
        self.get_logger().info("  - /camera/camera/color/image_rect_raw")
        self.get_logger().info("  - /grasp_6d/regenerate")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  - /grasp_6d/pointcloud")
        self.get_logger().info("  - /grasp_6d/mask")
    
    def _camera_info_callback(self, msg: CameraInfo):
        """Store camera info."""
        with self.data_lock:
            self.camera_info = {
                'K': list(msg.k),
                'width': msg.width,
                'height': msg.height,
                'D': list(msg.d),
                'P': list(msg.p),
            }
            # D405 baseline is 18mm
            if len(msg.p) >= 4 and msg.k[0] != 0:
                baseline = -msg.p[3] / msg.k[0] if msg.p[3] != 0 else 0.018
                self.camera_info['baseline'] = abs(baseline)
            else:
                self.camera_info['baseline'] = 0.018  # D405 default baseline
    
    def _regenerate_callback(self, msg: Empty):
        """Forward regenerate signal to FoundationStereo via ZMQ and file.
        
        Sets regenerate_in_progress flag to block reading stale PC files
        while FoundationStereo is running a fresh inference pass.
        """
        self.get_logger().info("Received regenerate signal from GraspGen, forwarding to FoundationStereo...")
        
        # Block reading stale PC files until new data arrives
        self.regenerate_in_progress = True
        
        # Forward via ZeroMQ on the stereo pub socket
        if self.stereo_pub_socket is not None:
            try:
                self.stereo_pub_socket.send_multipart([
                    b"regenerate",
                    json.dumps({'timestamp': time.time()}).encode('utf-8')
                ], zmq.NOBLOCK)
            except zmq.ZMQError as e:
                self.get_logger().warning(f"ZMQ regenerate send error: {e}")
        
        # Also write file-based signal as fallback
        try:
            regen_path = os.path.join(self.bridge_dir, 'regenerate_timestamp.txt')
            with open(regen_path, 'w') as f:
                f.write(str(time.time()))
        except Exception as e:
            self.get_logger().warning(f"File regenerate write error: {e}")
    
    def _stereo_callback(self, left_msg: Image, right_msg: Image):
        """Process synchronized stereo images."""
        left_img = self._ros_image_to_numpy(left_msg)
        right_img = self._ros_image_to_numpy(right_msg)
        
        if left_img is None or right_img is None:
            return
        
        with self.data_lock:
            self.stereo_left = left_img
            self.stereo_right = right_img
            camera_info = self.camera_info.copy() if self.camera_info else {}
        
        if not getattr(self, '_stereo_logged', False):
            self.get_logger().info(f"Stereo images received: {left_img.shape}")
            self._stereo_logged = True
        
        # Publish via ZeroMQ
        if self.stereo_pub_socket is not None:
            self._publish_stereo_zmq(left_img, right_img, camera_info)
        
        # Save to files
        self._save_stereo_files(left_img, right_img, camera_info)
    
    def _rgb_callback(self, msg: Image):
        """Process RGB image."""
        img = self._ros_image_to_numpy(msg, to_rgb=True)
        
        if img is None:
            return
        
        with self.data_lock:
            self.rgb_image = img
        
        # Publish via ZeroMQ
        if self.rgb_pub_socket is not None:
            self._publish_rgb_zmq(img)
        
        # Save to file
        self._save_rgb_file(img)
    
    def _ros_image_to_numpy(self, msg: Image, to_rgb: bool = False) -> np.ndarray:
        """Convert ROS Image to numpy array."""
        try:
            if msg.encoding == 'mono8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            elif msg.encoding == 'mono16':
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                img = (img / 256).astype(np.uint8)
            elif msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if to_rgb:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif msg.encoding == 'yuyv' or msg.encoding == 'yuv422':
                # D405 might output YUYV format
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
                img = cv2.cvtColor(img, cv2.COLOR_YUV2RGB_YUYV)
            else:
                self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
                return None
            return img
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return None
    
    def _publish_stereo_zmq(self, left: np.ndarray, right: np.ndarray, camera_info: dict):
        """Publish stereo images via ZeroMQ."""
        try:
            msg = {
                'timestamp': time.time(),
                'left_shape': left.shape,
                'right_shape': right.shape,
                'left_dtype': str(left.dtype),
                'right_dtype': str(right.dtype),
                'camera_info': camera_info,
            }
            
            self.stereo_pub_socket.send_multipart([
                b"stereo",
                json.dumps(msg).encode('utf-8'),
                left.tobytes(),
                right.tobytes()
            ], zmq.NOBLOCK)
            
        except zmq.ZMQError:
            pass
    
    def _publish_rgb_zmq(self, img: np.ndarray):
        """Publish RGB image via ZeroMQ."""
        try:
            msg = {
                'timestamp': time.time(),
                'shape': img.shape,
                'dtype': str(img.dtype),
            }
            
            self.rgb_pub_socket.send_multipart([
                b"rgb",
                json.dumps(msg).encode('utf-8'),
                img.tobytes()
            ], zmq.NOBLOCK)
            
        except zmq.ZMQError:
            pass
    
    def _save_stereo_files(self, left: np.ndarray, right: np.ndarray, camera_info: dict):
        """Save stereo images to files."""
        try:
            cv2.imwrite(os.path.join(self.bridge_dir, 'left.png'), left)
            cv2.imwrite(os.path.join(self.bridge_dir, 'right.png'), right)
            
            if camera_info:
                with open(os.path.join(self.bridge_dir, 'camera_info.json'), 'w') as f:
                    json.dump(camera_info, f)
            
            with open(os.path.join(self.bridge_dir, 'timestamp.txt'), 'w') as f:
                f.write(str(time.time()))
                
        except Exception as e:
            self.get_logger().warning(f"Failed to save stereo files: {e}")
    
    def _save_rgb_file(self, img: np.ndarray):
        """Save RGB image to file."""
        try:
            # Save as BGR for OpenCV
            if len(img.shape) == 3 and img.shape[2] == 3:
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img_bgr = img
            
            cv2.imwrite(os.path.join(self.bridge_dir, 'rgb.png'), img_bgr)
            
            with open(os.path.join(self.bridge_dir, 'rgb_timestamp.txt'), 'w') as f:
                f.write(str(time.time()))
                
        except Exception as e:
            self.get_logger().warning(f"Failed to save RGB file: {e}")
    
    def _check_processed_data(self):
        """Check for processed data from perception scripts and republish to ROS."""
        self._check_pointcloud()
        self._check_mask()
        
    
    def _check_pointcloud(self):
        """Check for point cloud from FoundationStereo.
        
        Drains the entire ZMQ queue and keeps only the latest point cloud.
        When ZMQ delivers data, updates last_pc_timestamp to prevent the
        file-based fallback from re-publishing the same stale data.
        
        During regeneration (regenerate_in_progress=True), skips reading
        file-based data to avoid re-publishing stale point clouds.
        """
        pc_data = None
        header = None
        
        # Try ZeroMQ first - drain entire queue, keep only latest
        if self.pc_sub_socket is not None:
            try:
                while True:
                    parts = self.pc_sub_socket.recv_multipart(zmq.NOBLOCK)
                    if len(parts) >= 3:
                        header = json.loads(parts[1].decode())
                        pc_data = parts[2]
            except zmq.Again:
                pass  # No more messages in queue
            except Exception as e:
                self.get_logger().warning(f"ZMQ PC receive error: {e}")
        
        # If we got data from ZMQ, update last_pc_timestamp and clear regenerate flag
        if pc_data is not None and header is not None:
            zmq_ts = header.get('timestamp', 0)
            if zmq_ts > 0:
                self.last_pc_timestamp = max(self.last_pc_timestamp, zmq_ts)
            # New data arrived via ZMQ, regeneration is complete
            self.regenerate_in_progress = False
        
        # Try file-based fallback
        # Check file even during regeneration - if file has new timestamp, regeneration is complete
        if pc_data is None:
            output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
            pc_timestamp_path = os.path.join(output_dir, 'pc_timestamp.txt')
            xyz_map_path = os.path.join(output_dir, 'xyz_map.npy')
            colors_path = os.path.join(output_dir, 'colors.npy')
            
            if os.path.exists(pc_timestamp_path) and os.path.exists(xyz_map_path):
                try:
                    with open(pc_timestamp_path, 'r') as f:
                        timestamp = float(f.read().strip())
                    
                    if timestamp > self.last_pc_timestamp:
                        # New data in file - regeneration is complete
                        self.regenerate_in_progress = False
                        self.last_pc_timestamp = timestamp
                        
                        # Load xyz_map and colors
                        xyz_map = np.load(xyz_map_path)
                        points = xyz_map.reshape(-1, 3)
                        
                        # Load colors if available
                        if os.path.exists(colors_path):
                            colors = np.load(colors_path)
                            colors = colors.reshape(-1, 3)
                        else:
                            colors = None
                        
                        valid_mask = (points[:, 2] > 0) & (points[:, 2] < 3.0)
                        points = points[valid_mask].astype(np.float32)
                        
                        # Apply same mask to colors
                        if colors is not None and len(colors) == len(valid_mask):
                            colors = colors[valid_mask].astype(np.uint8)
                        else:
                            colors = None
                        
                        if len(points) > 0:
                            # Create packed data with colors
                            import struct
                            data = []
                            for i in range(len(points)):
                                x, y, z = points[i]
                                if colors is not None and i < len(colors):
                                    r, g, b = colors[i]
                                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                                else:
                                    rgb = 0x808080  # Gray fallback
                                data.append(struct.pack('fffI', x, y, z, rgb))
                            
                            pc_data = b''.join(data)
                            header = {
                                'num_points': len(points),
                                'frame_id': 'camera_link',
                            }
                            if not getattr(self, '_pc_file_logged', False):
                                color_status = "with colors" if colors is not None else "gray (no colors)"
                                self.get_logger().info(f"Loaded PC from file: {len(points)} points {color_status}")
                                self._pc_file_logged = True
                            
                except Exception as e:
                    self.get_logger().warning(f"File PC load error: {e}")
        
        # Publish to ROS
        if pc_data is not None and header is not None:
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = header.get('frame_id', 'camera_link')
            
            msg.height = 1
            msg.width = header.get('num_points', len(pc_data) // 16)
            
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width
            msg.is_dense = True
            msg.data = pc_data
            
            self.pc_pub.publish(msg)
            if not getattr(self, '_pc_pub_logged', False):
                self.get_logger().info(f"Published point cloud: {msg.width} points")
                self._pc_pub_logged = True
    
    def _check_mask(self):
        """Check for mask from SAM2.
        
        Drains the entire ZMQ mask queue and keeps only the latest mask.
        SAM2 publishes masks at ~30fps, but the bridge timer runs at 20Hz,
        so stale masks accumulate in the queue. Without draining, GraspGen
        would receive outdated masks that appear 'fresh' by timestamp.
        """
        mask = None
        
        # Try ZeroMQ first - drain entire queue, keep only latest
        if self.mask_sub_socket is not None:
            drained = 0
            latest_mask_header = None
            try:
                while True:
                    parts = self.mask_sub_socket.recv_multipart(zmq.NOBLOCK)
                    if len(parts) >= 3:
                        latest_mask_header = json.loads(parts[1].decode())
                        H, W = latest_mask_header['height'], latest_mask_header['width']
                        mask = np.frombuffer(parts[2], dtype=np.uint8).reshape(H, W)
                        drained += 1
            except zmq.Again:
                pass  # No more messages in queue
            except Exception:
                pass
            if drained > 1:
                self.get_logger().debug(f'Drained {drained} masks from ZMQ queue, using latest')
            # Update file timestamp to prevent file fallback re-publishing same data
            if latest_mask_header is not None:
                zmq_ts = latest_mask_header.get('timestamp', 0)
                if zmq_ts > 0:
                    self.last_mask_timestamp = max(self.last_mask_timestamp, zmq_ts)
        
        # Try file-based
        if mask is None:
            mask_timestamp_path = os.path.join(self.bridge_dir, 'mask_timestamp.txt')
            mask_path = os.path.join(self.bridge_dir, 'mask.png')
            
            if os.path.exists(mask_timestamp_path):
                try:
                    with open(mask_timestamp_path, 'r') as f:
                        timestamp = float(f.read().strip())
                    
                    if timestamp > self.last_mask_timestamp:
                        self.last_mask_timestamp = timestamp
                        if os.path.exists(mask_path):
                            mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
                except Exception:
                    pass
        
        # Publish to ROS
        if mask is not None:
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.height, msg.width = mask.shape
            msg.encoding = 'mono8'
            msg.is_bigendian = False
            msg.step = msg.width
            msg.data = mask.tobytes()
            
            self.mask_pub.publish(msg)
            if not getattr(self, '_mask_pub_logged', False):
                self.get_logger().info(f"Published mask: {mask.shape}")
                self._mask_pub_logged = True
    
    def run(self):
        """Run the bridge."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ROS 6D Grasp Pipeline Bridge (Intel RealSense D405)")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("ZeroMQ Ports:")
        self.get_logger().info(f"  Stereo publish: {self.args.stereo_port}")
        self.get_logger().info(f"  Stereo PC sub:  {self.args.stereo_port + 1}")
        self.get_logger().info(f"  RGB publish:    {self.args.rgb_port}")
        self.get_logger().info(f"  Mask subscribe: {self.args.rgb_port + 1}")
        self.get_logger().info("")
        self.get_logger().info("File bridge directory:")
        self.get_logger().info(f"  {self.bridge_dir}")
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Bridge running. Press Ctrl+C to stop.")
        self.get_logger().info("=" * 60)
        
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            self._cleanup()
    
    def _cleanup(self):
        """Clean up resources."""
        if self.stereo_pub_socket:
            self.stereo_pub_socket.close()
        if self.rgb_pub_socket:
            self.rgb_pub_socket.close()
        if self.pc_sub_socket:
            self.pc_sub_socket.close()
        if self.mask_sub_socket:
            self.mask_sub_socket.close()
        if self.zmq_context:
            self.zmq_context.term()


def main():
    parser = argparse.ArgumentParser(description='ROS 6D Grasp Pipeline Bridge (D405)')
    parser.add_argument('--stereo-port', type=int, default=5555,
                       help='ZeroMQ port for stereo images (PC on port+1)')
    parser.add_argument('--rgb-port', type=int, default=5557,
                       help='ZeroMQ port for RGB images and masks')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = ROSBridge(args)
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
