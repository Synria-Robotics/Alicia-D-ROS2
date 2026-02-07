#!/usr/bin/env python3
"""
Segmentation - SAM2 (Intel RealSense D405)
Environment: sam2

Subscribes to RGB images, provides an interactive click UI for object selection,
runs SAM2 inference, and publishes the object mask.

Usage:
    conda activate sam2
    python d405_sam2.py [--model tiny|small|base|large]
    
Controls:
    - Left click: Select foreground point (object)
    - Right click: Select background point (not object)
    - 'r': Reset selection
    - 'q': Quit
"""

import os
import sys

# Suppress Qt font warnings
os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts')
import argparse
import time
import threading
import json
import numpy as np
import cv2
import torch
import logging
from collections import deque

# Add SAM2 to path - must be done before importing sam2
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SAM2_DIR = os.path.join(SCRIPT_DIR, '..', '..', 'Models', 'sam2')
sys.path.insert(0, SAM2_DIR)

# ROS 2 imports - handle gracefully for conda environments
ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except ImportError:
    pass
except AttributeError as e:
    # NumPy version conflict with cv_bridge
    print(f"[WARNING] ROS import issue (likely NumPy version conflict): {e}")
    print("[INFO] Running in bridge mode instead.")

# ZeroMQ for bridge communication
try:
    import zmq
    ZMQ_AVAILABLE = True
except ImportError:
    ZMQ_AVAILABLE = False
    print("[INFO] ZeroMQ not available. Install with: pip install pyzmq")


# SAM2 imports
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


class SAM2SegmentationNode:
    """
    SAM2-based interactive object segmentation for Intel RealSense D405.
    
    Can run in ROS mode or bridge mode (for conda environments with Python version mismatch).
    """
    
    # Model configurations - use relative config names for hydra
    MODEL_CONFIGS = {
        'tiny': ('configs/sam2/sam2_hiera_t.yaml', 'sam2_hiera_tiny.pt'),
        'small': ('configs/sam2/sam2_hiera_s.yaml', 'sam2_hiera_small.pt'),
        'base': ('configs/sam2/sam2_hiera_b+.yaml', 'sam2_hiera_base_plus.pt'),
        'large': ('configs/sam2/sam2_hiera_l.yaml', 'sam2_hiera_large.pt'),
    }
    
    def __init__(self, args):
        self.args = args
        self.predictor = None
        
        # Image buffer
        self.current_image = None
        self.image_lock = threading.Lock()
        self.new_image_available = False
        
        # Click prompts
        self.click_points = []  # List of (x, y) coordinates
        self.click_labels = []  # List of labels (1=foreground, 0=background)
        self.prompt_lock = threading.Lock()
        
        # Mask state
        self.current_mask = None
        self.mask_lock = threading.Lock()
        self.mask_updated = False
        
        # Image embedding state
        self.image_embedded = False
        self.embedding_lock = threading.Lock()
        
        # Bridge communication
        self.bridge_dir = os.path.join(SCRIPT_DIR, '.bridge_data')
        self.zmq_context = None
        self.zmq_sub_socket = None
        self.zmq_pub_socket = None
        self.last_timestamp = 0
        
        # Statistics
        self.inference_times = deque(maxlen=10)
        
        # Initialize model
        self._load_model()
        
        # Initialize communication
        self._init_communication()
        
        # Window name for OpenCV
        self.window_name = 'SAM2 Segmentation (D405) - Click to select object'
    
    def _load_model(self):
        """Load SAM2 model."""
        logging.info(f"Loading SAM2 model ({self.args.model})...")
        
        # Get config and checkpoint - use relative config path for hydra
        model_cfg, ckpt_name = self.MODEL_CONFIGS[self.args.model]
        checkpoint = os.path.join(SAM2_DIR, 'checkpoints', ckpt_name)
        
        if not os.path.exists(checkpoint):
            raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")
        
        # Build model - config_file should be relative path like 'configs/sam2/sam2_hiera_l.yaml'
        # The hydra compose function will find it in the sam2 package
        sam_model = build_sam2(model_cfg, checkpoint)
        self.predictor = SAM2ImagePredictor(sam_model)
        
        logging.info("SAM2 model loaded successfully!")
    
    def _init_communication(self):
        """Initialize ROS or bridge communication."""
        if ROS_AVAILABLE and not self.args.use_bridge:
            self._init_ros()
        else:
            self._init_bridge()
    
    def _init_ros(self):
        """Initialize ROS 2 node."""
        try:
            rclpy.init()
            self.node = rclpy.create_node('sam2_segmentation_node_d405')
            
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # QoS for latched mask - late subscribers get last message
            latched_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            
            # D405 specific topic
            self.image_sub = self.node.create_subscription(
                Image, '/camera/camera/color/image_rect_raw',
                self._image_callback, sensor_qos)
            
            self.mask_pub = self.node.create_publisher(
                Image, '/grasp_6d/mask', latched_qos)
            
            logging.info("ROS 2 node initialized (D405)")
            logging.info("Subscribing to: /camera/camera/color/image_rect_raw")
            logging.info("Publishing to: /grasp_6d/mask")
            self.use_ros = True
            
        except Exception as e:
            logging.warning(f"ROS initialization failed: {e}")
            logging.info("Falling back to bridge mode")
            self._init_bridge()
            self.use_ros = False
    
    def _init_bridge(self):
        """Initialize bridge communication (ZeroMQ + files)."""
        self.use_ros = False
        
        # File-based communication directory
        os.makedirs(self.bridge_dir, exist_ok=True)
        
        # ZeroMQ for receiving RGB and publishing masks
        if ZMQ_AVAILABLE:
            try:
                self.zmq_context = zmq.Context()
                
                # Subscriber for receiving RGB images from bridge
                self.zmq_rgb_socket = self.zmq_context.socket(zmq.SUB)
                self.zmq_rgb_socket.connect(f"tcp://localhost:{self.args.bridge_port}")
                self.zmq_rgb_socket.setsockopt_string(zmq.SUBSCRIBE, "rgb")
                self.zmq_rgb_socket.setsockopt(zmq.RCVTIMEO, 100)
                
                # Publisher for sending masks back to bridge
                self.zmq_pub_socket = self.zmq_context.socket(zmq.PUB)
                self.zmq_pub_socket.setsockopt(zmq.SNDHWM, 2)
                # Bind to a different port for masks
                mask_port = self.args.bridge_port + 1
                self.zmq_pub_socket.bind(f"tcp://*:{mask_port}")
                
                logging.info(f"Bridge mode: RGB sub port {self.args.bridge_port}, mask pub port {mask_port}")
                
            except Exception as e:
                logging.warning(f"ZeroMQ init failed: {e}")
                self.zmq_rgb_socket = None
                self.zmq_pub_socket = None
        else:
            self.zmq_rgb_socket = None
            self.zmq_pub_socket = None
        
        logging.info(f"Bridge mode: files in {self.bridge_dir}")
    
    def _image_callback(self, msg: 'Image'):
        """ROS image callback."""
        try:
            # Convert ROS Image to numpy
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                logging.warning(f"Unsupported encoding: {msg.encoding}")
                return
            
            with self.image_lock:
                self.current_image = img.copy()
                self.new_image_available = True
                
        except Exception as e:
            logging.error(f"Error processing image: {e}")
    
    def _receive_image_from_bridge(self) -> bool:
        """Receive RGB image from bridge. Returns True if new image available.
        
        Drains the ZMQ queue and keeps only the latest image to avoid
        displaying stale frames.
        """
        # Try ZeroMQ first
        if hasattr(self, 'zmq_rgb_socket') and self.zmq_rgb_socket is not None:
            latest_img = None
            received = False
            
            # Drain the queue to get the latest RGB image
            try:
                while True:
                    parts = self.zmq_rgb_socket.recv_multipart(zmq.NOBLOCK)
                    if len(parts) >= 3:
                        header = json.loads(parts[1].decode())
                        shape = tuple(header['shape'])
                        latest_img = np.frombuffer(parts[2], dtype=np.uint8).reshape(shape)
                        received = True
            except zmq.Again:
                pass  # No more messages in queue
            except Exception as e:
                logging.debug(f"ZMQ receive error: {e}")
            
            if latest_img is not None:
                with self.image_lock:
                    # Check if we need to reset embedding
                    if self.current_image is None or self.current_image.shape != latest_img.shape:
                        with self.embedding_lock:
                            self.image_embedded = False
                    self.current_image = latest_img
                    self.new_image_available = True
                return True
        
        # Fall back to file-based
        rgb_path = os.path.join(self.bridge_dir, 'rgb.png')
        timestamp_path = os.path.join(self.bridge_dir, 'rgb_timestamp.txt')
        
        if os.path.exists(timestamp_path):
            try:
                with open(timestamp_path, 'r') as f:
                    timestamp = float(f.read().strip())
                
                if timestamp > self.last_timestamp and os.path.exists(rgb_path):
                    self.last_timestamp = timestamp
                    img = cv2.imread(rgb_path)
                    if img is not None:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        with self.image_lock:
                            if self.current_image is None or self.current_image.shape != img.shape:
                                with self.embedding_lock:
                                    self.image_embedded = False
                            self.current_image = img
                            self.new_image_available = True
                        return True
            except Exception:
                pass
        
        return False
    
    def _publish_mask_ros(self, mask: np.ndarray):
        """Publish mask via ROS."""
        if not hasattr(self, 'node') or self.node is None:
            return
        
        try:
            # Convert mask to uint8 (0 or 255)
            # Handle boolean, float (0-1), or int masks
            if mask.dtype == bool:
                mask_uint8 = mask.astype(np.uint8) * 255
            elif mask.dtype in [np.float32, np.float64]:
                # Float mask - threshold at 0.5 and convert
                mask_uint8 = ((mask > 0.5).astype(np.uint8)) * 255
            else:
                # Integer mask - convert to binary (non-zero -> 255)
                mask_uint8 = ((mask > 0).astype(np.uint8)) * 255
            
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            
            msg.height, msg.width = mask_uint8.shape[:2]
            msg.encoding = 'mono8'
            msg.is_bigendian = False
            msg.step = msg.width
            msg.data = mask_uint8.tobytes()
            
            self.mask_pub.publish(msg)
        except Exception as e:
            logging.warning(f"ROS publish error: {e}")
    
    def _publish_mask_bridge(self, mask: np.ndarray):
        """Publish mask via bridge (ZeroMQ + files)."""
        # Convert mask to uint8 (0 or 255)
        if mask.dtype == bool:
            mask_uint8 = mask.astype(np.uint8) * 255
        elif mask.dtype in [np.float32, np.float64]:
            mask_uint8 = ((mask > 0.5).astype(np.uint8)) * 255
        else:
            mask_uint8 = ((mask > 0).astype(np.uint8)) * 255
        
        # Save to file
        try:
            mask_path = os.path.join(self.bridge_dir, 'mask.png')
            cv2.imwrite(mask_path, mask_uint8)
            
            with open(os.path.join(self.bridge_dir, 'mask_timestamp.txt'), 'w') as f:
                f.write(str(time.time()))
        except Exception as e:
            logging.warning(f"Failed to save mask: {e}")
        
        # Send via ZeroMQ
        if self.zmq_pub_socket is not None:
            try:
                header = {
                    'timestamp': time.time(),
                    'height': mask_uint8.shape[0],
                    'width': mask_uint8.shape[1],
                }
                self.zmq_pub_socket.send_multipart([
                    b"mask",
                    json.dumps(header).encode('utf-8'),
                    mask_uint8.tobytes()
                ], zmq.NOBLOCK)
            except Exception as e:
                pass
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events."""
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.prompt_lock:
                self.click_points.append((x, y))
                self.click_labels.append(1)
                self.mask_updated = False
            logging.info(f"Added foreground point at ({x}, {y})")
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            with self.prompt_lock:
                self.click_points.append((x, y))
                self.click_labels.append(0)
                self.mask_updated = False
            logging.info(f"Added background point at ({x}, {y})")
    
    def run_inference(self, image: np.ndarray, points: np.ndarray, labels: np.ndarray):
        """Run SAM2 inference."""
        start_time = time.time()
        
        with self.embedding_lock:
            if not self.image_embedded:
                self.predictor.set_image(image)
                self.image_embedded = True
        
        with torch.inference_mode():
            masks, scores, _ = self.predictor.predict(
                point_coords=points,
                point_labels=labels,
                multimask_output=True,
                return_logits=False
            )
        
        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)
        
        best_idx = np.argmax(scores)
        return masks[best_idx], scores[best_idx]
    
    def create_visualization(self, image: np.ndarray, mask: np.ndarray = None) -> np.ndarray:
        """Create visualization with mask overlay and click points."""
        vis = image.copy()
        
        if mask is not None:
            # Ensure mask is boolean
            if mask.dtype != bool:
                mask = mask.astype(bool)
            
            overlay = np.zeros_like(vis)
            overlay[mask] = [0, 255, 0]
            vis = cv2.addWeighted(vis, 0.7, overlay, 0.3, 0)
            
            contours, _ = cv2.findContours(mask.astype(np.uint8), 
                                          cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
        
        with self.prompt_lock:
            for (x, y), label in zip(self.click_points, self.click_labels):
                color = (0, 255, 0) if label == 1 else (0, 0, 255)
                cv2.circle(vis, (x, y), 8, color, -1)
                cv2.circle(vis, (x, y), 10, (255, 255, 255), 2)
        
        instructions = [
            "Left click: Add foreground point",
            "Right click: Add background point", 
            "R: Reset | Q: Quit"
        ]
        for i, text in enumerate(instructions):
            cv2.putText(vis, text, (10, 25 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, text, (10, 25 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        return vis
    
    def reset_selection(self):
        """Reset click prompts and mask."""
        with self.prompt_lock:
            self.click_points = []
            self.click_labels = []
        
        with self.mask_lock:
            self.current_mask = None
            self.mask_updated = False
        
        with self.embedding_lock:
            self.image_embedded = False
        
        logging.info("Selection reset")
    
    def run(self):
        """Main run loop."""
        if self.use_ros:
            self._run_ros_mode()
        elif self.args.webcam:
            self._run_webcam_mode()
        else:
            self._run_bridge_mode()
    
    def _run_ros_mode(self):
        """Run with ROS subscriptions."""
        logging.info("Running in ROS mode (D405)...")
        
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        
        spin_thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        spin_thread.start()
        
        self._run_main_loop(lambda: rclpy.ok())
        
        cv2.destroyAllWindows()
        self.node.destroy_node()
        rclpy.shutdown()
    
    def _run_webcam_mode(self):
        """Run with webcam as image source."""
        logging.info("Running in webcam mode...")
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logging.error("Could not open webcam")
            return
        
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                with self.image_lock:
                    # Check if image changed significantly
                    if self.current_image is None or \
                       self.current_image.shape != image.shape:
                        with self.embedding_lock:
                            self.image_embedded = False
                    self.current_image = image.copy()
                    self.new_image_available = True
                
                self._process_frame()
                
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.reset_selection()
                    
        finally:
            cap.release()
            cv2.destroyAllWindows()
    
    def _run_bridge_mode(self):
        """Run with bridge communication."""
        logging.info("Running in bridge mode (D405)...")
        logging.info("Waiting for RGB images from bridge...")
        logging.info("")
        logging.info("To send images, run the ROS bridge:")
        logging.info("  python d405_ros_bridge.py")
        logging.info("")
        logging.info("Or use --webcam flag to use webcam directly.")
        
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        
        try:
            while True:
                # Try to receive image from bridge
                self._receive_image_from_bridge()
                
                with self.image_lock:
                    if self.current_image is None:
                        # Show waiting message
                        blank = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(blank, "Waiting for RGB image from bridge...", 
                                   (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        cv2.imshow(self.window_name, blank)
                        key = cv2.waitKey(100) & 0xFF
                        if key == ord('q'):
                            break
                        continue
                
                self._process_frame()
                
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.reset_selection()
                    
        finally:
            cv2.destroyAllWindows()
            if self.zmq_pub_socket:
                self.zmq_pub_socket.close()
            if self.zmq_context:
                self.zmq_context.term()
    
    def _run_main_loop(self, is_running):
        """Main processing loop."""
        try:
            while is_running():
                with self.image_lock:
                    if self.current_image is None:
                        time.sleep(0.01)
                        continue
                
                self._process_frame()
                
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.reset_selection()
                    
        except KeyboardInterrupt:
            pass
    
    def _process_frame(self):
        """Process current frame - run inference and display."""
        with self.image_lock:
            if self.current_image is None:
                return
            image = self.current_image.copy()
        
        # Run inference if we have prompts
        with self.prompt_lock:
            has_prompts = len(self.click_points) > 0
            if has_prompts:
                points = np.array(self.click_points)
                labels = np.array(self.click_labels)
        
        if has_prompts and not self.mask_updated:
            mask, score = self.run_inference(image, points, labels)
            
            with self.mask_lock:
                self.current_mask = mask
                self.mask_updated = True
            
            avg_time = np.mean(self.inference_times) if self.inference_times else 0
            logging.info(f"Inference: {self.inference_times[-1]*1000:.1f}ms "
                       f"(avg: {avg_time*1000:.1f}ms), score: {score:.3f}")
        
        # Publish mask
        with self.mask_lock:
            if self.current_mask is not None:
                if self.use_ros:
                    self._publish_mask_ros(self.current_mask)
                else:
                    self._publish_mask_bridge(self.current_mask)
                current_mask = self.current_mask.copy()
            else:
                current_mask = None
        
        # Display
        vis = self.create_visualization(image, current_mask)
        vis_bgr = cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)
        cv2.imshow(self.window_name, vis_bgr)


def main():
    parser = argparse.ArgumentParser(description='SAM2 Segmentation Node (D405)')
    parser.add_argument('--model', type=str, default='large',
                       choices=['tiny', 'small', 'base', 'large'],
                       help='SAM2 model size')
    parser.add_argument('--webcam', action='store_true',
                       help='Use webcam as image source')
    parser.add_argument('--use_bridge', action='store_true',
                       help='Force bridge mode even if ROS is available')
    parser.add_argument('--bridge_port', type=int, default=5557,
                       help='ZeroMQ port for mask publishing')
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    
    logging.info("=" * 60)
    logging.info("SAM2 Segmentation Node (Intel RealSense D405)")
    logging.info("=" * 60)
    logging.info(f"Model: {args.model}")
    logging.info(f"Mode: {'webcam' if args.webcam else 'bridge/ROS'}")
    logging.info("=" * 60)
    
    node = SAM2SegmentationNode(args)
    node.run()


if __name__ == '__main__':
    main()
