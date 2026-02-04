#!/usr/bin/env python3
"""
ROS 2 Bridge for 6D Grasp Pipeline

This script runs in system Python (with ROS 2) and bridges camera data
to the perception scripts running in conda environments with different Python versions.

Bridges:
1. Stereo IR images -> FoundationStereo (perception_stereo.py)
2. RGB images -> SAM2 (segmentation_sam2.py)
3. Receives point clouds and masks, republishes to ROS topics

Usage:
    # In system Python environment (not conda)
    python3 ros_bridge.py [--stereo-port 5555] [--rgb-port 5557]
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
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
    Unified ROS 2 bridge for the 6D grasp pipeline.
    
    Handles:
    - Stereo IR images for FoundationStereo
    - RGB images for SAM2
    - Point cloud and mask republishing
    """
    
    def __init__(self, args):
        super().__init__('ros_grasp_bridge')
        self.args = args
        
        # Shared directory for file-based communication
        self.bridge_dir = os.path.join(os.path.dirname(__file__), '.bridge_data')
        os.makedirs(self.bridge_dir, exist_ok=True)
        
        # Data buffers
        self.stereo_left = None
        self.stereo_right = None
        self.rgb_image = None
        self.camera_info = None
        self.data_lock = threading.Lock()
        
        # Timestamps for file-based communication
        self.last_mask_timestamp = 0
        self.last_pc_timestamp = 0
        
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
        
        self.get_logger().info("ROS Bridge initialized")
        self.get_logger().info(f"Bridge directory: {self.bridge_dir}")
    
    def _init_zmq(self):
        """Initialize ZeroMQ sockets."""
        self.zmq_context = zmq.Context()
        
        # Publisher for stereo images (FoundationStereo subscribes)
        try:
            self.stereo_pub_socket = self.zmq_context.socket(zmq.PUB)
            self.stereo_pub_socket.setsockopt(zmq.SNDHWM, 2)
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
        
        # === Stereo IR Subscribers ===
        self.left_sub = message_filters.Subscriber(
            self, Image, '/camera/left_ir/image_raw', qos_profile=sensor_qos)
        self.right_sub = message_filters.Subscriber(
            self, Image, '/camera/right_ir/image_raw', qos_profile=sensor_qos)
        
        self.stereo_sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=5, slop=0.05)
        self.stereo_sync.registerCallback(self._stereo_callback)
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/left_ir/camera_info',
            self._camera_info_callback, sensor_qos)
        
        # === RGB Subscriber ===
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self._rgb_callback, sensor_qos)
        
        # === Publishers (for republishing processed data) ===
        self.pc_pub = self.create_publisher(
            PointCloud2, '/grasp_6d/pointcloud', reliable_qos)
        
        self.mask_pub = self.create_publisher(
            Image, '/grasp_6d/mask', reliable_qos)
        
        # Timer to check for processed data
        self.check_timer = self.create_timer(0.05, self._check_processed_data)
        
        self.get_logger().info("ROS interfaces initialized")
        self.get_logger().info("Subscriptions:")
        self.get_logger().info("  - /camera/left_ir/image_raw")
        self.get_logger().info("  - /camera/right_ir/image_raw")
        self.get_logger().info("  - /camera/color/image_raw")
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
            if len(msg.p) >= 4 and msg.k[0] != 0:
                baseline = -msg.p[3] / msg.k[0] if msg.p[3] != 0 else 0.05
                self.camera_info['baseline'] = abs(baseline)
            else:
                self.camera_info['baseline'] = 0.05
    
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
        """Check for point cloud from FoundationStereo."""
        pc_data = None
        header = None
        
        # Try ZeroMQ first
        if self.pc_sub_socket is not None:
            try:
                parts = self.pc_sub_socket.recv_multipart(zmq.NOBLOCK)
                if len(parts) >= 3:
                    header = json.loads(parts[1].decode())
                    pc_data = parts[2]
            except zmq.Again:
                pass
            except Exception as e:
                self.get_logger().warning(f"ZMQ PC receive error: {e}")
        
        # Try file-based fallback
        if pc_data is None:
            output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
            pc_timestamp_path = os.path.join(output_dir, 'pc_timestamp.txt')
            xyz_map_path = os.path.join(output_dir, 'xyz_map.npy')
            
            if os.path.exists(pc_timestamp_path) and os.path.exists(xyz_map_path):
                try:
                    with open(pc_timestamp_path, 'r') as f:
                        timestamp = float(f.read().strip())
                    
                    if timestamp > self.last_pc_timestamp:
                        self.last_pc_timestamp = timestamp
                        
                        # Load xyz_map and create point cloud data
                        xyz_map = np.load(xyz_map_path)
                        points = xyz_map.reshape(-1, 3)
                        valid_mask = (points[:, 2] > 0) & (points[:, 2] < 3.0)
                        points = points[valid_mask].astype(np.float32)
                        
                        if len(points) > 0:
                            # Create packed data
                            import struct
                            data = []
                            for i in range(len(points)):
                                x, y, z = points[i]
                                rgb = 0x808080  # Gray color
                                data.append(struct.pack('fffI', x, y, z, rgb))
                            
                            pc_data = b''.join(data)
                            header = {
                                'num_points': len(points),
                                'frame_id': 'camera_link',
                            }
                            if not getattr(self, '_pc_file_logged', False):
                                self.get_logger().info(f"Loaded PC from file: {len(points)} points")
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
        """Check for mask from SAM2."""
        mask = None
        
        # Try ZeroMQ first
        if self.mask_sub_socket is not None:
            try:
                parts = self.mask_sub_socket.recv_multipart(zmq.NOBLOCK)
                if len(parts) >= 3:
                    header = json.loads(parts[1].decode())
                    H, W = header['height'], header['width']
                    mask = np.frombuffer(parts[2], dtype=np.uint8).reshape(H, W)
            except zmq.Again:
                pass
            except Exception:
                pass
        
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
        self.get_logger().info("ROS 6D Grasp Pipeline Bridge")
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
    parser = argparse.ArgumentParser(description='ROS 6D Grasp Pipeline Bridge')
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
