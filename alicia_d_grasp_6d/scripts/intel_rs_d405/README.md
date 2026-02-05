# 6D Grasp Pipeline for Intel RealSense D405

This folder contains the 6D grasp pipeline implementation for the Intel RealSense D405 camera

## Key Differences from Gemini 335

1. **Topic Names**: D405 uses `/camera/camera/` prefix instead of `/camera/`
2. **Frame Names**: `camera_infra1_optical_frame` instead of `camera_left_ir_optical_frame`
3. **Camera Parameters**:
   - IR resolution: 848x480
   - RGB resolution: 848x480 (same as IR, unlike Gemini 335)
   - Stereo baseline: 18mm (vs 50mm for Gemini 335)
   - Different intrinsic parameters
4. **RGB Point Cloud Coloring**: The point cloud is colored using RGB camera data


## ROS Topics

### Input Topics (from D405 camera)
- `/camera/camera/infra1/image_rect_raw` - Left IR image
- `/camera/camera/infra2/image_rect_raw` - Right IR image
- `/camera/camera/color/image_rect_raw` - RGB image
- `/camera/camera/depth/image_rect_raw` - Depth image
- `/camera/camera/infra1/camera_info` - IR camera info
- `/camera/camera/color/camera_info` - Color camera info

### Output Topics
- `/grasp_6d/pointcloud` - Colored point cloud
- `/grasp_6d/mask` - Segmentation mask
- `/grasp_6d/grasp_poses` - Generated grasp poses
- `/grasp_6d/grasp_confidences` - Grasp confidence scores

## Pipeline Components

### 1. ROS Bridge (`d405_ros_bridge.py`)
Bridges camera data between ROS and conda environments.

```bash
# Run in system Python (ROS 2 environment)
python3 d405_ros_bridge.py
```

### 2. FoundationStereo Depth Estimation (`d405_foundationstereo.py`)
Runs FoundationStereo for depth estimation and generates RGB-colored point cloud.

**Key Feature**: Colors the point cloud using RGB camera data.

```bash
# Run in foundation_stereo conda environment
conda activate foundation_stereo
python d405_foundationstereo.py --visualize
```

### 3. SAM2 Segmentation (`d405_sam2.py`)
Interactive object segmentation using SAM2.

```bash
# Run in sam2 conda environment
conda activate sam2
python d405_sam2.py --model large
```

Controls:
- Left click: Add foreground point
- Right click: Add background point
- 'r': Reset selection
- 'q': Quit

### 4. GraspGen Grasp Generation (`d405_graspgen.py`)
Generates grasp poses using GraspGen and visualizes colored point cloud in meshcat.

**Key Feature**: Displays colored point cloud in meshcat visualization.

```bash
# Run in GraspGen conda environment
conda activate GraspGen
# First start meshcat server in another terminal: meshcat-server
python d405_graspgen.py
```

### 5. Grasp Execution (`d405_execution.py`)
Executes grasps using MoveIt 2.

```bash
# Run in system Python (ROS 2 environment)
python3 d405_execution.py
```

## Quick Start

### Terminal 1: Start D405 camera
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true enable_color:=true
```

### Terminal 2: ROS Bridge
```bash
python3 d405_ros_bridge.py
```

### Terminal 3: FoundationStereo
```bash
conda activate foundation_stereo
python d405_foundationstereo.py
```

### Terminal 4: SAM2 Segmentation
```bash
conda activate sam2
python d405_sam2.py
```

### Terminal 5: Meshcat Server
```bash
meshcat-server
```

### Terminal 6: GraspGen
```bash
conda activate GraspGen
python d405_graspgen.py
```

### Terminal 7: Grasp Execution
```bash
python3 d405_execution.py
```

## File Structure

```
intel_rs_d405/
├── d405_ros_bridge.py        # ROS bridge for camera data
├── d405_foundationstereo.py  # Depth estimation with RGB coloring
├── d405_sam2.py              # SAM2 segmentation
├── d405_graspgen.py          # Grasp generation with colored visualization
├── d405_execution.py         # Grasp execution
├── README.md                 # This file
└── utils/
    ├── __init__.py
    ├── camera_utils.py       # D405 camera parameters
    └── transform_utils.py    # Coordinate transformations
```

## Calibration

Hand-eye calibration result should be saved at:
```
alicia_d_calibation/config/hand_eye_calibration_result.yaml
```

This calibration connects `gripper_center` to `camera_link` in the TF tree.

## Notes

- The D405 has a shorter baseline (18mm) compared to Gemini 335 (50mm), which affects depth accuracy
- RGB and IR have the same resolution (848x480), simplifying the IR-to-RGB projection
- The D405 is a compact camera with close-range depth sensing capability
