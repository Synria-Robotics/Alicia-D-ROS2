#!/usr/bin/env python3
"""
Perception - FoundationStereo (Intel RealSense D405)
Environment: foundation_stereo (Python 3.11)

Subscribes to D405 left/right IR images via bridge, runs FoundationStereo 
inference, and publishes a 3D point cloud COLORED USING RGB CAMERA.

Since the foundation_stereo conda environment uses Python 3.11, and ROS 2
Humble binaries are built for Python 3.10, this script communicates with ROS
via a bridge (d405_ros_bridge.py) using ZeroMQ or shared files.

Key difference from Gemini 335:
- Uses RGB camera to color the point cloud output
- D405 has same resolution for RGB and IR (848x480)

Usage:
    # Terminal 1: Start the ROS bridge (system Python)
    python d405_ros_bridge.py
    
    # Terminal 2: Run this script (conda environment)
    conda activate foundation_stereo
    python d405_foundationstereo.py [--visualize]
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
import torch
import logging
from collections import deque

# Add FoundationStereo to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FOUNDATION_STEREO_DIR = os.path.join(SCRIPT_DIR, '..', '..', 'Models', 'FoundationStereo')
sys.path.insert(0, FOUNDATION_STEREO_DIR)

from omegaconf import OmegaConf
from core.utils.utils import InputPadder
from Utils import set_logging_format, set_seed, vis_disparity, depth2xyzmap, toOpen3dCloud
from core.foundation_stereo import FoundationStereo

# ZeroMQ for inter-process communication
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[INFO] ZeroMQ not available. Install with: pip install pyzmq")
    print("[INFO] Will use file-based communication as fallback.")


# Import default camera parameters for D405
try:
    from utils.camera_utils import DEFAULT_IR_K, DEFAULT_BASELINE, DEFAULT_RGB_K, DEFAULT_T_IR_TO_RGB
except ImportError:
    # Fallback defaults if utils not available (D405 specific values)
    DEFAULT_IR_K = np.array([
        [428.6961364746094, 0.0, 420.7202453613281],
        [0.0, 428.6961364746094, 239.13221740722656],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    DEFAULT_RGB_K = np.array([
        [435.9206237792969, 0.0, 420.70172119140625],
        [0.0, 435.5009460449219, 237.35459899902344],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    DEFAULT_T_IR_TO_RGB = np.array([
        [1.000, -0.001,  0.001,  0.000],
        [0.001,  1.000,  0.001, -0.000],
        [-0.001, -0.001,  1.000, -0.000],
        [0.000,  0.000,  0.000,  1.000]
    ], dtype=np.float32)
    DEFAULT_BASELINE = 0.018


class FoundationStereoNode:
    """
    FoundationStereo depth estimation and point cloud generation.
    
    Communicates with ROS via bridge using ZeroMQ or shared files.
    Camera intrinsics are loaded from bridge_data/camera_info.json if available,
    otherwise falls back to default values.
    
    KEY FEATURE: Colors point cloud using RGB camera image.
    """
    
    def __init__(self, args):
        self.args = args
        self.model = None
        self.K = DEFAULT_IR_K.copy()
        self.K_rgb = DEFAULT_RGB_K.copy()
        self.T_ir_to_rgb = DEFAULT_T_IR_TO_RGB.copy()
        self.baseline = DEFAULT_BASELINE
        
        # Image buffers
        self.left_image = None
        self.right_image = None
        self.rgb_image = None  # RGB image for coloring
        self.image_lock = threading.Lock()
        self.new_image_available = False
        
        # Bridge communication
        self.zmq_context = None
        self.zmq_sub_socket = None
        self.zmq_pub_socket = None
        self.zmq_rgb_socket = None  # For receiving RGB images
        self.bridge_dir = os.path.join(SCRIPT_DIR, '.bridge_data')
        self.last_timestamp = 0
        self.last_rgb_timestamp = 0
        # Initialize from existing file to prevent spurious triggers from stale files
        self.last_regen_file_timestamp = self._read_regen_file_timestamp()
        
        # Regenerate signal (triggered by GraspGen via bridge)
        self.regenerate_requested = threading.Event()
        
        # Statistics
        self.inference_times = deque(maxlen=10)
        self.frame_count = 0
        
        # Initialize model
        self._load_model()
        
        # Initialize communication
        self._init_communication()
    
    def _load_model(self):
        """Load FoundationStereo model."""
        logging.info("Loading FoundationStereo model...")
        
        ckpt_dir = self.args.ckpt_dir
        cfg_path = os.path.join(os.path.dirname(ckpt_dir), 'cfg.yaml')
        
        if not os.path.exists(cfg_path):
            logging.warning(f"Config file not found at {cfg_path}, using default config")
            cfg = OmegaConf.create({
                'vit_size': 'vitl',
                'valid_iters': self.args.valid_iters,
            })
        else:
            cfg = OmegaConf.load(cfg_path)
        
        if 'vit_size' not in cfg:
            cfg['vit_size'] = 'vitl'
        
        cfg['valid_iters'] = self.args.valid_iters
        
        self.model = FoundationStereo(cfg)
        
        ckpt = torch.load(ckpt_dir, map_location='cuda')
        logging.info(f"Checkpoint global_step: {ckpt.get('global_step', 'N/A')}, epoch: {ckpt.get('epoch', 'N/A')}")
        self.model.load_state_dict(ckpt['model'])
        
        self.model.cuda()
        self.model.eval()
        
        logging.info("Model loaded successfully!")
    
    def _init_communication(self):
        """Initialize ZeroMQ or file-based communication."""
        if ZMQ_AVAILABLE:
            try:
                self.zmq_context = zmq.Context()
                
                # Subscriber for receiving stereo images and regenerate signals from bridge
                self.zmq_sub_socket = self.zmq_context.socket(zmq.SUB)
                self.zmq_sub_socket.connect(f"tcp://localhost:{self.args.bridge_port}")
                self.zmq_sub_socket.setsockopt_string(zmq.SUBSCRIBE, "stereo")
                self.zmq_sub_socket.setsockopt_string(zmq.SUBSCRIBE, "regenerate")
                self.zmq_sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
                
                # Subscriber for receiving RGB images from bridge
                self.zmq_rgb_socket = self.zmq_context.socket(zmq.SUB)
                self.zmq_rgb_socket.connect(f"tcp://localhost:{self.args.rgb_port}")
                self.zmq_rgb_socket.setsockopt_string(zmq.SUBSCRIBE, "rgb")
                self.zmq_rgb_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
                
                # Publisher for sending point clouds back to bridge
                self.zmq_pub_socket = self.zmq_context.socket(zmq.PUB)
                self.zmq_pub_socket.bind(f"tcp://*:{self.args.bridge_port + 1}")
                
                logging.info(f"ZeroMQ initialized: stereo sub port {self.args.bridge_port}, "
                           f"rgb sub port {self.args.rgb_port}, pub port {self.args.bridge_port + 1}")
                
            except Exception as e:
                logging.warning(f"ZeroMQ initialization failed: {e}")
                self.zmq_sub_socket = None
                self.zmq_pub_socket = None
                self.zmq_rgb_socket = None
        
        # File-based fallback
        os.makedirs(self.bridge_dir, exist_ok=True)
        logging.info(f"File-based communication: {self.bridge_dir}")
    
    def _receive_images_zmq(self) -> bool:
        """Receive stereo images via ZeroMQ. Returns True if new images received.
        
        Drains the entire ZMQ queue and keeps only the latest stereo pair.
        This prevents using stale frames that accumulated during inference.
        """
        if self.zmq_sub_socket is None:
            return False
        
        received = False
        latest_left = None
        latest_right = None
        latest_camera_info = {}
        drained_count = 0
        
        # Drain the queue to get the latest stereo images
        # Also detect regenerate signals mixed in the same socket
        try:
            while True:
                parts = self.zmq_sub_socket.recv_multipart(zmq.NOBLOCK)
                topic = parts[0].decode()
                
                if topic == "regenerate":
                    self.regenerate_requested.set()
                    logging.debug("Received regenerate signal via ZMQ")
                    continue
                
                if topic == "stereo" and len(parts) >= 4:
                    header = json.loads(parts[1].decode())
                    left_data = parts[2]
                    right_data = parts[3]
                    
                    left_shape = tuple(header['left_shape'])
                    right_shape = tuple(header['right_shape'])
                    
                    latest_left = np.frombuffer(left_data, dtype=np.uint8).reshape(left_shape)
                    latest_right = np.frombuffer(right_data, dtype=np.uint8).reshape(right_shape)
                    
                    camera_info = header.get('camera_info', {})
                    if camera_info:
                        latest_camera_info = camera_info
                    
                    drained_count += 1
                    received = True
        except zmq.Again:
            pass  # No more messages in queue
        except Exception as e:
            logging.warning(f"ZMQ receive error: {e}")
        
        if drained_count > 1:
            logging.debug(f"Drained {drained_count} stereo frames from ZMQ queue, using latest")
        
        # Update with the latest stereo images
        if latest_left is not None and latest_right is not None:
            if latest_camera_info:
                if 'K' in latest_camera_info:
                    self.K = np.array(latest_camera_info['K'], dtype=np.float32).reshape(3, 3)
                if 'baseline' in latest_camera_info:
                    self.baseline = latest_camera_info['baseline']
            
            with self.image_lock:
                self.left_image = latest_left
                self.right_image = latest_right
                self.new_image_available = True
        
        return received
    
    def _receive_rgb_zmq(self) -> bool:
        """Receive RGB image via ZeroMQ for coloring point cloud."""
        if self.zmq_rgb_socket is None:
            return False
        
        received = False
        latest_rgb = None
        
        # Drain the queue to get the latest RGB image
        try:
            while True:
                parts = self.zmq_rgb_socket.recv_multipart(zmq.NOBLOCK)
                if len(parts) >= 3:
                    header = json.loads(parts[1].decode())
                    shape = tuple(header['shape'])
                    latest_rgb = np.frombuffer(parts[2], dtype=np.uint8).reshape(shape)
                    received = True
        except zmq.Again:
            pass  # No more messages in queue
        except Exception as e:
            logging.debug(f"ZMQ RGB receive error: {e}")
        
        # Update with the latest RGB if we got any
        if latest_rgb is not None:
            with self.image_lock:
                self.rgb_image = latest_rgb
        
        return received
    
    def _receive_images_file(self) -> bool:
        """Receive images via shared files. Returns True if new images available."""
        timestamp_file = os.path.join(self.bridge_dir, 'timestamp.txt')
        
        if not os.path.exists(timestamp_file):
            return False
        
        try:
            with open(timestamp_file, 'r') as f:
                timestamp = float(f.read().strip())
            
            if timestamp <= self.last_timestamp:
                return False
            
            self.last_timestamp = timestamp
            
            # Load stereo images
            left_path = os.path.join(self.bridge_dir, 'left.png')
            right_path = os.path.join(self.bridge_dir, 'right.png')
            
            if not os.path.exists(left_path) or not os.path.exists(right_path):
                return False
            
            left_img = cv2.imread(left_path)
            right_img = cv2.imread(right_path)
            
            if left_img is None or right_img is None:
                return False
            
            # Convert BGR to RGB if needed
            if len(left_img.shape) == 3:
                left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2RGB)
            elif len(left_img.shape) == 2:
                # Grayscale - convert to RGB
                left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2RGB)
            
            # Load camera info
            camera_info_path = os.path.join(self.bridge_dir, 'camera_info.json')
            if os.path.exists(camera_info_path):
                with open(camera_info_path, 'r') as f:
                    camera_info = json.load(f)
                if 'K' in camera_info:
                    self.K = np.array(camera_info['K'], dtype=np.float32).reshape(3, 3)
                if 'baseline' in camera_info:
                    self.baseline = camera_info['baseline']
            
            with self.image_lock:
                self.left_image = left_img
                self.right_image = right_img
                self.new_image_available = True
            
            return True
            
        except Exception as e:
            logging.warning(f"File receive error: {e}")
            return False
    
    def _receive_rgb_file(self) -> bool:
        """Receive RGB image via shared files for coloring point cloud."""
        rgb_timestamp_path = os.path.join(self.bridge_dir, 'rgb_timestamp.txt')
        rgb_path = os.path.join(self.bridge_dir, 'rgb.png')
        
        # Also check if we can read RGB directly even without timestamp (if file exists and is recent)
        if not os.path.exists(rgb_path):
            return False
        
        try:
            # Check timestamp if available
            new_timestamp = False
            if os.path.exists(rgb_timestamp_path):
                with open(rgb_timestamp_path, 'r') as f:
                    timestamp = float(f.read().strip())
                
                if timestamp > self.last_rgb_timestamp:
                    self.last_rgb_timestamp = timestamp
                    new_timestamp = True
            
            # Always try to read RGB if we have no RGB yet, or if timestamp is new
            if new_timestamp or self.rgb_image is None:
                rgb_img = cv2.imread(rgb_path)
                if rgb_img is not None:
                    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
                    with self.image_lock:
                        self.rgb_image = rgb_img
                    return True
                    
        except Exception as e:
            logging.debug(f"File RGB receive error: {e}")
            
        return False
    
    def _color_pointcloud_with_rgb(self, xyz_map: np.ndarray, rgb_img: np.ndarray) -> np.ndarray:
        """
        Color the point cloud using RGB camera image.
        
        Args:
            xyz_map: XYZ point cloud map (H, W, 3) in IR frame
            rgb_img: RGB image (H_rgb, W_rgb, 3)
            
        Returns:
            colors: Color array (H*W, 3) corresponding to xyz_map
        """
        H, W = xyz_map.shape[:2]
        H_rgb, W_rgb = rgb_img.shape[:2]
        
        # Flatten xyz_map for vectorized operations
        points = xyz_map.reshape(-1, 3)
        num_pixels = H * W
        
        # Valid points mask (z > 0)
        valid_mask = points[:, 2] > 0.001
        
        # Transform points from IR frame to RGB frame
        points_homo = np.hstack([points, np.ones((num_pixels, 1))])  # (N, 4)
        points_rgb = (self.T_ir_to_rgb @ points_homo.T).T[:, :3]  # (N, 3)
        
        # Project to RGB image coordinates
        fx_rgb, fy_rgb = self.K_rgb[0, 0], self.K_rgb[1, 1]
        cx_rgb, cy_rgb = self.K_rgb[0, 2], self.K_rgb[1, 2]
        
        z_rgb = np.clip(points_rgb[:, 2], 0.001, None)
        u_rgb = (fx_rgb * points_rgb[:, 0] / z_rgb + cx_rgb).astype(np.int32)
        v_rgb = (fy_rgb * points_rgb[:, 1] / z_rgb + cy_rgb).astype(np.int32)
        
        # Check bounds
        valid_bounds = (u_rgb >= 0) & (u_rgb < W_rgb) & (v_rgb >= 0) & (v_rgb < H_rgb)
        valid_color = valid_mask & valid_bounds
        
        # Initialize colors (default gray for invalid points)
        colors = np.ones((num_pixels, 3), dtype=np.uint8) * 128
        
        # Sample colors from RGB image for valid points
        valid_indices = np.where(valid_color)[0]
        if len(valid_indices) > 0:
            colors[valid_indices] = rgb_img[v_rgb[valid_indices], u_rgb[valid_indices]]
        
        return colors.reshape(H, W, 3)
    
    def _publish_pointcloud_zmq(self, xyz_map: np.ndarray, colors: np.ndarray, timestamp: float = None):
        """Publish colored point cloud via ZeroMQ.
        
        Args:
            xyz_map: Point cloud XYZ map (H, W, 3)
            colors: RGB colors (H, W, 3)
            timestamp: Shared timestamp for consistency with file-based path.
                       If None, uses current time.
        """
        if self.zmq_pub_socket is None:
            logging.debug("ZMQ pub socket not available, skipping publish")
            return
        
        try:
            # Ensure xyz_map is 3D
            if xyz_map.ndim != 3 or xyz_map.shape[2] != 3:
                logging.warning(f"xyz_map has unexpected shape: {xyz_map.shape}")
                return
            
            H, W = xyz_map.shape[:2]
            num_pixels = H * W
            
            # Flatten xyz_map
            points = xyz_map.reshape(num_pixels, 3)
            valid_mask = (points[:, 2] > 0) & (points[:, 2] <= self.args.z_far)
            
            # Flatten colors
            colors_flat = colors.reshape(num_pixels, 3)
            
            # Apply valid_mask to both
            points = points[valid_mask].astype(np.float32)
            colors_valid = colors_flat[valid_mask].astype(np.uint8)
            
            # Pack data: XYZRGB as float32 + uint32
            data = []
            for i in range(len(points)):
                x, y, z = points[i]
                r, g, b = colors_valid[i]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                data.append(struct.pack('fffI', x, y, z, rgb))
            
            pc_data = b''.join(data)
            
            header = {
                'timestamp': timestamp if timestamp is not None else time.time(),
                'num_points': len(points),
                'frame_id': 'camera_link',
            }
            
            self.zmq_pub_socket.send_multipart([
                b"pointcloud",
                json.dumps(header).encode('utf-8'),
                pc_data
            ], zmq.NOBLOCK)
            
            if not getattr(self, '_pc_pub_logged', False):
                logging.info(f"Point cloud publishing via ZeroMQ ({len(points)} colored points)")
                self._pc_pub_logged = True
            
        except Exception as e:
            logging.warning(f"ZMQ publish error: {e}")
    
    def _save_pointcloud_file(self, xyz_map: np.ndarray, colors: np.ndarray, depth: np.ndarray, timestamp: float = None):
        """Save colored point cloud to shared files.
        
        Args:
            xyz_map: Point cloud XYZ map (H, W, 3)
            colors: RGB colors (H, W, 3)
            depth: Depth map in meters (H, W)
            timestamp: Shared timestamp for consistency with ZMQ path.
                       If None, uses current time.
        """
        output_dir = os.path.join(SCRIPT_DIR, 'outputs')
        os.makedirs(output_dir, exist_ok=True)
        
        try:
            # Save depth, xyz_map, and colors
            np.save(os.path.join(output_dir, 'depth_meter.npy'), depth)
            np.save(os.path.join(output_dir, 'xyz_map.npy'), xyz_map)
            np.save(os.path.join(output_dir, 'colors.npy'), colors)  # Save colors for file-based transfer
            
            # Save colored point cloud
            import open3d as o3d
            
            if xyz_map.ndim != 3 or xyz_map.shape[2] != 3:
                logging.warning(f"xyz_map has unexpected shape for file save: {xyz_map.shape}")
                return
            
            H, W = xyz_map.shape[:2]
            num_pixels = H * W
            points = xyz_map.reshape(num_pixels, 3)
            colors_flat = colors.reshape(num_pixels, 3)
            
            valid_mask = (points[:, 2] > 0) & (points[:, 2] <= self.args.z_far)
            
            pcd = toOpen3dCloud(points[valid_mask], colors_flat[valid_mask])
            if self.args.denoise_cloud:
                pcd, _ = pcd.remove_radius_outlier(nb_points=self.args.denoise_nb_points,
                                                   radius=self.args.denoise_radius)
            o3d.io.write_point_cloud(os.path.join(output_dir, 'pointcloud.ply'), pcd)
            
            # Write timestamp to signal new data (use shared timestamp for consistency)
            with open(os.path.join(output_dir, 'pc_timestamp.txt'), 'w') as f:
                f.write(str(timestamp if timestamp is not None else time.time()))
                
        except Exception as e:
            logging.warning(f"Failed to save point cloud: {e}")
    
    def run_inference(self, left_img: np.ndarray, right_img: np.ndarray):
        """
        Run FoundationStereo inference on stereo image pair.
        
        Args:
            left_img: Left IR image (H, W, 3) RGB or (H, W) grayscale
            right_img: Right IR image (H, W, 3) RGB or (H, W) grayscale
            
        Returns:
            depth: Depth map in meters (H, W)
            xyz_map: XYZ point cloud (H, W, 3)
            disp: Disparity map (H, W)
        """
        # Ensure RGB format
        if len(left_img.shape) == 2:
            left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
            right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2RGB)
        
        H, W = left_img.shape[:2]
        
        # Scale images if needed
        scale = self.args.scale
        if scale < 1.0:
            left_img = cv2.resize(left_img, None, fx=scale, fy=scale)
            right_img = cv2.resize(right_img, None, fx=scale, fy=scale)
        
        H_scaled, W_scaled = left_img.shape[:2]
        
        # Convert to tensor
        img0 = torch.as_tensor(left_img).cuda().float()[None].permute(0, 3, 1, 2)
        img1 = torch.as_tensor(right_img).cuda().float()[None].permute(0, 3, 1, 2)
        
        # Pad for network
        padder = InputPadder(img0.shape, divis_by=32, force_square=False)
        img0_padded, img1_padded = padder.pad(img0, img1)
        
        # Run inference
        start_time = time.time()
        with torch.cuda.amp.autocast(True):
            with torch.no_grad():
                if not self.args.hiera:
                    disp = self.model.forward(img0_padded, img1_padded, 
                                             iters=self.args.valid_iters, test_mode=True)
                else:
                    disp = self.model.run_hierachical(img0_padded, img1_padded,
                                                     iters=self.args.valid_iters, 
                                                     test_mode=True, small_ratio=0.5)
        
        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)
        
        # Unpad and convert to numpy
        disp = padder.unpad(disp.float())
        disp = disp.data.cpu().numpy().reshape(H_scaled, W_scaled)
        
        # Remove non-overlapping regions
        if self.args.remove_invisible:
            yy, xx = np.meshgrid(np.arange(disp.shape[0]), np.arange(disp.shape[1]), indexing='ij')
            us_right = xx - disp
            invalid = us_right < 0
            disp[invalid] = np.inf
        
        # Scale intrinsics if images were scaled
        K = self.K.copy()
        K[:2] *= scale
        
        # Convert disparity to depth
        depth = K[0, 0] * self.baseline / disp
        depth[np.isinf(depth)] = 0
        depth[depth > self.args.z_far] = 0
        depth[depth < 0.01] = 0
        
        # Generate XYZ map
        xyz_map = depth2xyzmap(depth, K)
        
        # Scale back to original resolution if needed
        if scale < 1.0:
            depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)
            xyz_map = cv2.resize(xyz_map, (W, H), interpolation=cv2.INTER_NEAREST)
            disp = cv2.resize(disp, (W, H), interpolation=cv2.INTER_NEAREST)
        
        return depth, xyz_map, disp
    
    def visualize_frame(self, left_img: np.ndarray, disp: np.ndarray, 
                       xyz_map: np.ndarray, depth: np.ndarray, colors: np.ndarray):
        """Visualize disparity and colored point cloud using Open3D."""
        import open3d as o3d
        
        # Handle grayscale images
        if left_img.ndim == 2:
            left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
        
        # Create visualization
        disp_vis = vis_disparity(disp)
        combined = np.concatenate([left_img, disp_vis], axis=1)
        
        cv2.imshow('Left Image | Disparity', cv2.cvtColor(combined, cv2.COLOR_RGB2BGR))
        
        # Create Open3D point cloud with RGB colors
        H, W = xyz_map.shape[:2]
        points = xyz_map.reshape(-1, 3)
        colors_flat = colors.reshape(-1, 3) / 255.0  # Normalize to 0-1
        valid_mask = (points[:, 2] > 0) & (points[:, 2] <= self.args.z_far)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[valid_mask])
        pcd.colors = o3d.utility.Vector3dVector(colors_flat[valid_mask])
        
        # Denoise if requested
        if self.args.denoise_cloud:
            pcd, _ = pcd.remove_radius_outlier(nb_points=self.args.denoise_nb_points,
                                               radius=self.args.denoise_radius)
        
        # Visualize
        logging.info("Visualizing colored point cloud. Press ESC or Q to close.")
        vis = o3d.visualization.Visualizer()
        vis.create_window('Colored Point Cloud (D405)', width=800, height=600)
        vis.add_geometry(pcd)
        vis.get_render_option().point_size = 1.0
        vis.get_render_option().background_color = np.array([0.5, 0.5, 0.5])
        vis.run()
        vis.destroy_window()
        cv2.destroyAllWindows()
    
    def _read_regen_file_timestamp(self) -> float:
        """Read the current regenerate file timestamp (for initialization)."""
        regen_path = os.path.join(os.path.join(SCRIPT_DIR, '.bridge_data'), 'regenerate_timestamp.txt')
        if os.path.exists(regen_path):
            try:
                with open(regen_path, 'r') as f:
                    return float(f.read().strip())
            except Exception:
                pass
        return 0.0

    def _check_regenerate_file(self) -> bool:
        """Check for file-based regenerate signal from bridge."""
        regen_path = os.path.join(self.bridge_dir, 'regenerate_timestamp.txt')
        if not os.path.exists(regen_path):
            return False
        try:
            with open(regen_path, 'r') as f:
                timestamp = float(f.read().strip())
            if timestamp > self.last_regen_file_timestamp:
                self.last_regen_file_timestamp = timestamp
                return True
        except Exception:
            pass
        return False
    
    def _wait_for_regenerate(self):
        """Wait for a regenerate signal from GraspGen (via bridge).
        
        Blocks until the regenerate signal is received via ZMQ or file.
        During the wait, we keep draining ZMQ queues so stereo/RGB images
        stay fresh.
        """
        logging.info("")
        logging.info("Waiting for regenerate signal from GraspGen...")
        logging.info("(Press Ctrl+C to quit)")
        
        self.regenerate_requested.clear()
        
        while not self.regenerate_requested.is_set():
            # Drain stereo + regenerate signals from ZMQ
            self._receive_images_zmq()
            # Drain RGB
            self._receive_rgb_zmq()
            # Check file-based regenerate
            if self._check_regenerate_file():
                self.regenerate_requested.set()
                break
            time.sleep(0.05)
        
        logging.info("Regenerate signal received!")
        
        # Sync file timestamp to prevent spurious triggers on the next wait.
        # If the signal came via ZMQ, the file may also have been written with
        # the same timestamp. Update last_regen_file_timestamp so the next
        # _check_regenerate_file() call won't trigger on the same file.
        regen_path = os.path.join(self.bridge_dir, 'regenerate_timestamp.txt')
        try:
            if os.path.exists(regen_path):
                with open(regen_path, 'r') as f:
                    ts = float(f.read().strip())
                self.last_regen_file_timestamp = max(self.last_regen_file_timestamp, ts)
        except Exception:
            pass
    
    def run(self):
        """Main run loop.
        
        First run: waits for stereo+RGB images, runs inference, publishes point cloud.
        Subsequent runs: waits for a regenerate signal from GraspGen (via bridge),
        then grabs latest images, re-runs inference, and re-publishes.
        """
        logging.info("=" * 60)
        logging.info("FoundationStereo Perception Node (Intel RealSense D405)")
        logging.info("=" * 60)
        logging.info(f"Waiting for images from bridge...")
        logging.info(f"  ZeroMQ: {'enabled' if ZMQ_AVAILABLE else 'disabled'}")
        logging.info(f"  Stereo bridge port: {self.args.bridge_port}")
        logging.info(f"  RGB bridge port: {self.args.rgb_port}")
        logging.info(f"  File fallback: {self.bridge_dir}")
        logging.info("=" * 60)
        logging.info("")
        logging.info("FEATURE: Point cloud will be colored using RGB camera")
        logging.info("NOTE: Regeneration is triggered by GraspGen (no manual Enter needed)")
        logging.info("")
        logging.info("Make sure d405_ros_bridge.py is running in another terminal:")
        logging.info("  python d405_ros_bridge.py")
        logging.info("")
        logging.info("=" * 60)
        
        first_run = True
        rgb_wait_logged = False
        
        try:
            while True:
                # Reset for new inference cycle
                rgb_wait_logged = False
                
                # Wait for fresh images
                logging.info("Waiting for stereo and RGB images...")
                
                # Clear old data to ensure we get fresh images
                with self.image_lock:
                    self.new_image_available = False
                    self.rgb_image = None
                
                # Wait until we have both stereo and RGB
                got_stereo = False
                got_rgb = False
                wait_start = time.time()
                
                while not (got_stereo and got_rgb):
                    # Try to receive stereo images
                    received_zmq = self._receive_images_zmq()
                    received_file = False
                    
                    if not received_zmq:
                        received_file = self._receive_images_file()
                    
                    if received_zmq or received_file:
                        got_stereo = True
                    
                    # Try to receive RGB image for coloring
                    self._receive_rgb_zmq()
                    self._receive_rgb_file()
                    
                    with self.image_lock:
                        if self.rgb_image is not None:
                            got_rgb = True
                    
                    # Log waiting status
                    if not got_rgb and not rgb_wait_logged and got_stereo:
                        logging.info("Stereo received, waiting for RGB image...")
                        rgb_wait_logged = True
                    
                    # Timeout check (30 seconds)
                    if time.time() - wait_start > 30.0:
                        if not got_stereo:
                            logging.warning("Timeout waiting for stereo images")
                        if not got_rgb:
                            logging.warning("Timeout waiting for RGB, will use grayscale")
                            got_rgb = True  # Continue without RGB
                        break
                    
                    time.sleep(0.01)
                
                # Get images
                with self.image_lock:
                    if self.left_image is None or self.right_image is None:
                        logging.warning("No stereo images available, waiting...")
                        time.sleep(0.1)
                        continue
                    
                    left_img = self.left_image.copy()
                    right_img = self.right_image.copy()
                    rgb_img = self.rgb_image.copy() if self.rgb_image is not None else None
                    self.new_image_available = False
                
                self.frame_count += 1
                
                # Log source and RGB status
                logging.info(f"Processing frame {self.frame_count}...")
                if rgb_img is not None:
                    logging.info(f"  Stereo: {left_img.shape[1]}x{left_img.shape[0]}, RGB: {rgb_img.shape[1]}x{rgb_img.shape[0]}")
                else:
                    logging.warning(f"  Stereo: {left_img.shape[1]}x{left_img.shape[0]}, RGB: not available (using grayscale)")
                
                # Run inference
                logging.info("Running FoundationStereo inference...")
                depth, xyz_map, disp = self.run_inference(left_img, right_img)
                
                # Color point cloud using RGB
                if rgb_img is not None:
                    colors = self._color_pointcloud_with_rgb(xyz_map, rgb_img)
                    logging.info(f"  Point cloud colored with RGB (colors range: {colors.min()}-{colors.max()})")
                else:
                    # Fallback: use grayscale from IR
                    if left_img.ndim == 2:
                        gray = left_img
                    else:
                        gray = cv2.cvtColor(left_img, cv2.COLOR_RGB2GRAY)
                    colors = np.stack([gray, gray, gray], axis=-1)
                    logging.info("  Point cloud colored with grayscale (no RGB)")
                
                # Visualize only on first run if --visualize flag is set
                if first_run and self.args.visualize:
                    logging.info("")
                    logging.info("Opening point cloud visualization...")
                    logging.info("Close the visualization window to continue.")
                    self.visualize_frame(left_img, disp, xyz_map, depth, colors)
                    logging.info("Visualization closed.")
                
                # On first run without visualization, wait a moment for GraspGen to be ready
                if first_run and not self.args.visualize:
                    logging.info("")
                    logging.info("First inference complete. Waiting 2 seconds for GraspGen to be ready...")
                    time.sleep(2.0)
                
                first_run = False
                
                # Publish/save colored point cloud with a shared timestamp
                # to ensure ZMQ and file paths use the exact same value.
                publish_timestamp = time.time()
                self._publish_pointcloud_zmq(xyz_map, colors, publish_timestamp)
                self._save_pointcloud_file(xyz_map, colors, depth, publish_timestamp)
                
                logging.info("")
                logging.info("=" * 50)
                logging.info("Point cloud published successfully!")
                logging.info("=" * 50)
                
                # Wait for regenerate signal from GraspGen (via bridge)
                # This replaces the old input() wait
                self._wait_for_regenerate()
                logging.info("")
                logging.info("Regenerating point cloud...")
                
        except KeyboardInterrupt:
            logging.info("Shutting down...")
        finally:
            if self.zmq_sub_socket:
                self.zmq_sub_socket.close()
            if self.zmq_rgb_socket:
                self.zmq_rgb_socket.close()
            if self.zmq_pub_socket:
                self.zmq_pub_socket.close()
            if self.zmq_context:
                self.zmq_context.term()


def main():
    parser = argparse.ArgumentParser(description='FoundationStereo Perception Node (D405)')
    parser.add_argument('--ckpt_dir', type=str,
                       default=os.path.join(FOUNDATION_STEREO_DIR, 'pretrained_models/11-33-40/model_best_bp2.pth'),
                       help='Path to pretrained model checkpoint')
    parser.add_argument('--bridge_port', type=int, default=5555,
                       help='ZeroMQ bridge port for stereo (subscribes to this, publishes to port+1)')
    parser.add_argument('--rgb_port', type=int, default=5557,
                       help='ZeroMQ bridge port for RGB images')
    parser.add_argument('--scale', type=float, default=1.0,
                       help='Downscale factor for input images (must be <= 1)')
    parser.add_argument('--hiera', action='store_true',
                       help='Use hierarchical inference (for high-res images)')
    parser.add_argument('--valid_iters', type=int, default=32,
                       help='Number of refinement iterations')
    parser.add_argument('--z_far', type=float, default=3.0,
                       help='Maximum depth to include in point cloud (meters)')
    parser.add_argument('--remove_invisible', action='store_true', default=True,
                       help='Remove non-overlapping regions between left/right views')
    parser.add_argument('--denoise_cloud', action='store_true', default=True,
                       help='Apply radius outlier removal to point cloud')
    parser.add_argument('--denoise_nb_points', type=int, default=30,
                       help='Number of points for radius outlier removal')
    parser.add_argument('--denoise_radius', type=float, default=0.03,
                       help='Radius for outlier removal (meters)')
    parser.add_argument('--visualize', action='store_true',
                       help='Visualize first frame with Open3D')
    
    args = parser.parse_args()
    
    set_logging_format()
    set_seed(0)
    torch.autograd.set_grad_enabled(False)
    
    node = FoundationStereoNode(args)
    node.run()


if __name__ == '__main__':
    main()
