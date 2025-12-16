# Alicia-D ROS2 Usage Guide

This document provides complete usage instructions and examples for Alicia-D ROS2.

# Simulation

For gripper type of `50mm`, use the default value:

```
ros2 launch alicia_d_moveit demo.launch.py
```

For gripper type of `100mm`, specify the gripper type:

```
ros2 launch alicia_d_moveit demo.launch.py gripper_type:=100mm
```

# Real Robot MoveIt

Basic launch (using default parameters):
```
ros2 launch alicia_d_moveit real_robot.launch.py
```

Specify gripper type:
```
ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm
```

Specify serial port (optional if auto-detection is enabled):
```
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=100mm \
    port:=/dev/ttyACM0
```

Specify motion speed (degrees per second):
```
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=50mm \
    port:=/dev/ttyACM0 \
    speed_deg_s:=30
```

## Launch Parameters

### MoveIt Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gripper_type` | `50mm` | Gripper type: `50mm` or `100mm` |
| `port` | `''` (empty string) | Serial port device path, e.g., `/dev/ttyACM0`. Leave empty for auto-detection |
| `speed_deg_s` | `20` | Default speed for joint movements (degrees per second) |

# Standalone Driver Node (No MoveIt)

The standalone driver node provides basic robot control functionality without MoveIt dependency.

## Launch Standalone Driver

Basic launch (using default parameters):
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py
```

Specify serial port:
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py port:=/dev/ttyACM0
```

Specify default speed:
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py \
    port:=/dev/ttyACM0 \
    default_speed_deg_s:=30.0
```

## Standalone Driver Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `''` (empty string) | Serial port device path, e.g., `/dev/ttyACM0`. Leave empty for auto-detection |
| `default_speed_deg_s` | `20.0` | Default speed for joint movements (degrees per second), range: 4.39-439.45 |

## Standalone Driver Topics

### Subscribed Topics

- `/joint_commands` (sensor_msgs/JointState) - Send joint position and velocity commands
- `/demonstration` (std_msgs/Bool) - Enable/disable hand-guiding mode (zero torque)
- `/zero_calibrate` (std_msgs/Bool) - Execute zero calibration

### Published Topics

- `/joint_states` (sensor_msgs/JointState) - Publish current joint states (position, velocity)

## Usage Examples

### Enable Hand-Guiding Mode (Zero Torque)

After enabling hand-guiding mode, you can manually drag the robot for teaching:

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

### Disable Hand-Guiding Mode (Restore Full Torque)

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
```

### Zero Calibration

Step 1: Disable torque first (enter hand-guiding mode)
```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

Step 2: Manually move the robot to the desired zero position

Step 3: Execute zero calibration (torque will be automatically restored after calibration)

**⚠️ Warning: This operation is irreversible. Skip this step if not necessary.**
```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "{data: true}"
```

### Send Joint Commands

Send joint position and velocity commands via `/joint_commands` topic:

**Example 1: Move to zero position with default speed (20 deg/s)**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0]
"
```

**Example 2: Move with specific speed (30 deg/s = 0.524 rad/s)**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [0.785, 0.785, 0.0, 0.0, 0.0, 0.0, 500.0]
velocity: [0.524, 0.524, 0.524, 0.524, 0.524, 0.524, 0.0]
"
```

**Example 3: Move with high speed (100 deg/s = 1.745 rad/s)**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [1.57, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [1.745, 1.745, 1.745, 1.745, 1.745, 1.745, 0.0]
"
```

**Note on Units:**

The `/joint_commands` topic uses `sensor_msgs/JointState`, which follows ROS standard conventions:
- Joint positions are in **radians (rad)** (ROS standard)
- Velocities in the `velocity` field are in **radians per second (rad/s)** (ROS standard)

**Important:** Although the driver internally uses degrees per second (`speed_deg_s`), the ROS message standard requires rad/s. The driver automatically converts rad/s → deg/s internally, so you must provide velocities in rad/s to follow ROS conventions.

**Behavior:**
- If velocity values are provided in rad/s, the driver converts them to deg/s and uses the maximum as the common speed for all joints
- If velocity is not provided, the `default_speed_deg_s` parameter value (in deg/s) will be used
- Gripper position range is 0-1000 (0 = fully open, 1000 = fully closed)

**Speed Conversion Reference (for /joint_commands topic):**
- 20 deg/s = 0.349 rad/s → use `velocity: [0.349, 0.349, ...]`
- 30 deg/s = 0.524 rad/s → use `velocity: [0.524, 0.524, ...]`
- 40 deg/s = 0.698 rad/s → use `velocity: [0.698, 0.698, ...]`
- 100 deg/s = 1.745 rad/s → use `velocity: [1.745, 1.745, ...]`

**Quick conversion formula:** `rad/s = deg/s × π / 180` or `rad/s = deg/s × 0.0174533`

