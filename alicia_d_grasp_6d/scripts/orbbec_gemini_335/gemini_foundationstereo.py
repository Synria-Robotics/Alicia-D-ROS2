#!/usr/bin/env python3
"""
Perception - FoundationStereo
Environment: foundation_stereo (Python 3.11)

Subscribes to Gemini 335 left/right IR images via bridge, runs FoundationStereo 
inference, and publishes a 3D point cloud.

Since the foundation_stereo conda environment uses Python 3.11, and ROS 2
Humble binaries are built for Python 3.10, this script communicates with ROS
via a bridge (ros_image_bridge.py) using ZeroMQ or shared files.

Usage:
    # Terminal 1: Start the ROS bridge (system Python)
    python ros_image_bridge.py
    
    # Terminal 2: Run this script (conda environment)
    conda activate foundation_stereo
    python perception_stereo.py [--visualize]
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


# Import default camera parameters
try:
    from utils.camera_utils import DEFAULT_IR_K, DEFAULT_BASELINE
except ImportError:
    # Fallback defaults if utils not available
    DEFAULT_IR_K = np.array([
        [411.666748046875, 0.0, 420.0250244140625],
        [0.0, 411.666748046875, 240.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    DEFAULT_BASELINE = 0.05


class FoundationStereoNode:
    """
    FoundationStereo depth estimation and point cloud generation.
    
    Communicates with ROS via bridge using ZeroMQ or shared files.
    Camera intrinsics are loaded from bridge_data/camera_info.json if available,
    otherwise falls back to default values.
    """
    
    def __init__(self, args):
        self.args = args
        self.model = None
        self.K = DEFAULT_IR_K.copy()
        self.baseline = DEFAULT_BASELINE
        
        # Image buffers
        self.left_image = None
        self.right_image = None
        self.image_lock = threading.Lock()
        self.new_image_available = False
        
        # Bridge communication
        self.zmq_context = None
        self.zmq_sub_socket = None
        self.zmq_pub_socket = None
        self.bridge_dir = os.path.join(SCRIPT_DIR, '.bridge_data')
        self.last_timestamp = 0
        
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
                
                # Subscriber for receiving images from bridge
                self.zmq_sub_socket = self.zmq_context.socket(zmq.SUB)
                self.zmq_sub_socket.connect(f"tcp://localhost:{self.args.bridge_port}")
                self.zmq_sub_socket.setsockopt_string(zmq.SUBSCRIBE, "stereo")
                self.zmq_sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
                
                # Publisher for sending point clouds back to bridge
                self.zmq_pub_socket = self.zmq_context.socket(zmq.PUB)
                self.zmq_pub_socket.bind(f"tcp://*:{self.args.bridge_port + 1}")
                
                logging.info(f"ZeroMQ initialized: sub port {self.args.bridge_port}, pub port {self.args.bridge_port + 1}")
                
            except Exception as e:
                logging.warning(f"ZeroMQ initialization failed: {e}")
                self.zmq_sub_socket = None
                self.zmq_pub_socket = None
        
        # File-based fallback
        os.makedirs(self.bridge_dir, exist_ok=True)
        logging.info(f"File-based communication: {self.bridge_dir}")
    
    def _receive_images_zmq(self) -> bool:
        """Receive images via ZeroMQ. Returns True if new images received."""
        if self.zmq_sub_socket is None:
            return False
        
        try:
            parts = self.zmq_sub_socket.recv_multipart(zmq.NOBLOCK)
            if len(parts) >= 4:
                topic = parts[0].decode()
                header = json.loads(parts[1].decode())
                left_data = parts[2]
                right_data = parts[3]
                
                # Reconstruct images
                left_shape = tuple(header['left_shape'])
                right_shape = tuple(header['right_shape'])
                
                left_img = np.frombuffer(left_data, dtype=np.uint8).reshape(left_shape)
                right_img = np.frombuffer(right_data, dtype=np.uint8).reshape(right_shape)
                
                # Update camera info if provided
                camera_info = header.get('camera_info', {})
                if camera_info:
                    if 'K' in camera_info:
                        self.K = np.array(camera_info['K'], dtype=np.float32).reshape(3, 3)
                    if 'baseline' in camera_info:
                        self.baseline = camera_info['baseline']
                
                with self.image_lock:
                    self.left_image = left_img
                    self.right_image = right_img
                    self.new_image_available = True
                
                return True
                
        except zmq.Again:
            return False
        except Exception as e:
            logging.warning(f"ZMQ receive error: {e}")
            return False
        
        return False
    
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
            
            # Load images
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
    
    def _publish_pointcloud_zmq(self, xyz_map: np.ndarray, rgb_img: np.ndarray):
        """Publish point cloud via ZeroMQ."""
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
            
            # Prepare colors - ensure same number of pixels
            if rgb_img is not None:
                # Handle grayscale images (IR cameras)
                if rgb_img.ndim == 2:
                    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2RGB)
                
                # Resize to match xyz_map exactly
                rgb_resized = cv2.resize(rgb_img, (W, H), interpolation=cv2.INTER_LINEAR)
                colors_flat = rgb_resized.reshape(num_pixels, 3)
            else:
                colors_flat = np.ones((num_pixels, 3), dtype=np.uint8) * 128
            
            # Verify shapes match before indexing
            if len(valid_mask) != len(colors_flat):
                logging.warning(f"Shape mismatch: valid_mask={len(valid_mask)}, colors={len(colors_flat)}")
                # Use gray colors as fallback
                colors_flat = np.ones((num_pixels, 3), dtype=np.uint8) * 128
            
            # Now apply valid_mask to both
            points = points[valid_mask].astype(np.float32)
            colors = colors_flat[valid_mask].astype(np.uint8)
            
            # Pack data: XYZRGB as float32 + uint32
            data = []
            for i in range(len(points)):
                x, y, z = points[i]
                r, g, b = colors[i]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                data.append(struct.pack('fffI', x, y, z, rgb))
            
            pc_data = b''.join(data)
            
            header = {
                'timestamp': time.time(),
                'num_points': len(points),
                'frame_id': 'camera_link',
            }
            
            self.zmq_pub_socket.send_multipart([
                b"pointcloud",
                json.dumps(header).encode('utf-8'),
                pc_data
            ], zmq.NOBLOCK)
            
            if not getattr(self, '_pc_pub_logged', False):
                logging.info(f"Point cloud publishing via ZeroMQ ({len(points)} points)")
                self._pc_pub_logged = True
            
        except Exception as e:
            logging.warning(f"ZMQ publish error: {e}")
    
    def _save_pointcloud_file(self, xyz_map: np.ndarray, rgb_img: np.ndarray, depth: np.ndarray):
        """Save point cloud to shared files."""
        output_dir = os.path.join(SCRIPT_DIR, 'outputs')
        os.makedirs(output_dir, exist_ok=True)
        
        try:
            # Save depth and xyz_map
            np.save(os.path.join(output_dir, 'depth_meter.npy'), depth)
            np.save(os.path.join(output_dir, 'xyz_map.npy'), xyz_map)
            
            # Save point cloud
            import open3d as o3d
            
            if xyz_map.ndim != 3 or xyz_map.shape[2] != 3:
                logging.warning(f"xyz_map has unexpected shape for file save: {xyz_map.shape}")
                return
            
            H, W = xyz_map.shape[:2]
            num_pixels = H * W
            points = xyz_map.reshape(num_pixels, 3)
            
            # Resize rgb_img to match xyz_map resolution exactly
            if rgb_img is not None:
                # Handle grayscale images (IR cameras)
                if rgb_img.ndim == 2:
                    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2RGB)
                
                rgb_resized = cv2.resize(rgb_img, (W, H), interpolation=cv2.INTER_LINEAR)
                colors = rgb_resized.reshape(num_pixels, 3)
            else:
                colors = np.ones((num_pixels, 3), dtype=np.uint8) * 128
            
            valid_mask = (points[:, 2] > 0) & (points[:, 2] <= self.args.z_far)
            
            pcd = toOpen3dCloud(points[valid_mask], colors[valid_mask])
            if self.args.denoise_cloud:
                pcd, _ = pcd.remove_radius_outlier(nb_points=self.args.denoise_nb_points,
                                                   radius=self.args.denoise_radius)
            o3d.io.write_point_cloud(os.path.join(output_dir, 'pointcloud.ply'), pcd)
            
            # Write timestamp to signal new data
            with open(os.path.join(output_dir, 'pc_timestamp.txt'), 'w') as f:
                f.write(str(time.time()))
                
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
                       xyz_map: np.ndarray, depth: np.ndarray):
        """Visualize disparity and point cloud using Open3D."""
        import open3d as o3d
        
        # Handle grayscale images
        if left_img.ndim == 2:
            left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
        
        # Create visualization
        disp_vis = vis_disparity(disp)
        combined = np.concatenate([left_img, disp_vis], axis=1)
        
        cv2.imshow('Left Image | Disparity', cv2.cvtColor(combined, cv2.COLOR_RGB2BGR))
        
        # Create Open3D point cloud
        points = xyz_map.reshape(-1, 3)
        colors = left_img.reshape(-1, 3) / 255.0
        valid_mask = (points[:, 2] > 0) & (points[:, 2] <= self.args.z_far)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[valid_mask])
        pcd.colors = o3d.utility.Vector3dVector(colors[valid_mask])
        
        # Denoise if requested
        if self.args.denoise_cloud:
            pcd, _ = pcd.remove_radius_outlier(nb_points=self.args.denoise_nb_points,
                                               radius=self.args.denoise_radius)
        
        # Visualize
        logging.info("Visualizing point cloud. Press ESC or Q to close.")
        vis = o3d.visualization.Visualizer()
        vis.create_window('Point Cloud', width=800, height=600)
        vis.add_geometry(pcd)
        vis.get_render_option().point_size = 1.0
        vis.get_render_option().background_color = np.array([0.5, 0.5, 0.5])
        vis.run()
        vis.destroy_window()
        cv2.destroyAllWindows()
    
    def run(self):
        """Main run loop."""
        logging.info("=" * 60)
        logging.info("FoundationStereo Perception Node")
        logging.info("=" * 60)
        logging.info(f"Waiting for images from bridge...")
        logging.info(f"  ZeroMQ: {'enabled' if ZMQ_AVAILABLE else 'disabled'}")
        logging.info(f"  Bridge port: {self.args.bridge_port}")
        logging.info(f"  File fallback: {self.bridge_dir}")
        logging.info("=" * 60)
        logging.info("")
        logging.info("Make sure ros_image_bridge.py is running in another terminal:")
        logging.info("  python ros_image_bridge.py")
        logging.info("")
        logging.info("=" * 60)
        
        visualized = False
        
        try:
            while True:
                # Try to receive images
                received_zmq = self._receive_images_zmq()
                received_file = False
                
                if not received_zmq:
                    received_file = self._receive_images_file()
                
                if not received_zmq and not received_file:
                    time.sleep(0.01)
                    continue
                
                # Get images
                with self.image_lock:
                    if not self.new_image_available:
                        continue
                    left_img = self.left_image.copy()
                    right_img = self.right_image.copy()
                    self.new_image_available = False
                
                self.frame_count += 1
                
                # Log source
                if not getattr(self, '_frame_logged', False):
                    source = "ZMQ" if received_zmq else "file"
                    logging.info(f"Receiving images via {source} ({left_img.shape[1]}x{left_img.shape[0]})")
                    self._frame_logged = True
                
                # Run inference
                depth, xyz_map, disp = self.run_inference(left_img, right_img)
                
                # Publish/save point cloud
                self._publish_pointcloud_zmq(xyz_map, left_img)
                self._save_pointcloud_file(xyz_map, left_img, depth)
                
                # Visualize first frame if requested
                if self.args.visualize and not visualized:
                    self.visualize_frame(left_img, disp, xyz_map, depth)
                    visualized = True
                    logging.info("Visualization closed. Continuing to process...")
                
        except KeyboardInterrupt:
            logging.info("Shutting down...")
        finally:
            if self.zmq_sub_socket:
                self.zmq_sub_socket.close()
            if self.zmq_pub_socket:
                self.zmq_pub_socket.close()
            if self.zmq_context:
                self.zmq_context.term()


def main():
    parser = argparse.ArgumentParser(description='FoundationStereo Perception Node')
    parser.add_argument('--ckpt_dir', type=str,
                       default=os.path.join(FOUNDATION_STEREO_DIR, 'pretrained_models/11-33-40/model_best_bp2.pth'),
                       help='Path to pretrained model checkpoint')
    parser.add_argument('--bridge_port', type=int, default=5555,
                       help='ZeroMQ bridge port (subscribes to this, publishes to port+1)')
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
