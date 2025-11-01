# Alicia-D Real Robot Integration with MoveIt

This guide explains how to use the updated `alicia_d_driver` and `alicia_d_moveit` packages to control the real Alicia-D robot with MoveIt.

## Overview

The system has been updated to connect the real Alicia-D robot with MoveIt using ros2_control. The architecture now includes:

1. **Hardware Interface Plugin**: A ros2_control SystemInterface that communicates directly with the robot via serial
2. **ROS2 Control Controllers**: Joint trajectory controllers for the arm and gripper
3. **MoveIt Integration**: Full MoveIt support for motion planning and execution on the real robot

## Architecture

```
MoveIt → Joint Trajectory Controllers → ros2_control → Hardware Interface → Serial → Robot
                                                ↓
                                        Joint State Publisher
                                                ↓
                                           /joint_states
```

## Key Components

### 1. Hardware Interface (`alicia_d_hardware_interface.cpp`)
- Implements the ros2_control `SystemInterface`
- Handles serial communication with the robot
- Converts between ROS joint states and hardware protocol
- Supports firmware version detection and multiple gripper types

### 2. ROS2 Control Configuration
- **Controller Manager**: Manages all controllers
- **Joint State Broadcaster**: Publishes joint states to `/joint_states`
- **Alicia_controller**: Joint trajectory controller for the 6-DOF arm
- **Gripper_controller**: Joint trajectory controller for the gripper

### 3. Launch Files
- **real_robot.launch.py**: Launch file for controlling the real robot with MoveIt
- **demo.launch.py**: Original demo with fake execution (simulation)

## Installation & Setup

### 1. Build the Packages

```bash
cd ~/alicia_ws
colcon build --packages-select alicia_d_driver alicia_d_moveit
source install/setup.bash
```

### 2. Set Serial Port Permissions

Make sure your user has access to the serial port:

```bash
sudo usermod -a -G dialout $USER
```

Then **log out and log back in** for the changes to take effect.

Alternatively, set permissions temporarily:

```bash
sudo chmod 666 /dev/ttyCH343USB0
```

### 3. Verify Serial Connection

Check that the robot is connected:

```bash
ls -l /dev/ttyCH343USB0
# or
ls -l /dev/ttyUSB*
```

## Usage

### Controlling the Real Robot with MoveIt

Launch the complete system for real robot control:

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```

**Launch Arguments:**
- `robot_version`: Robot version (default: `v5_6`, options: `v5_5`, `v5_6`)
- `gripper_type`: Gripper type (default: `50mm`, options: `50mm`, `100mm`)
- `port`: Serial port (default: `/dev/ttyCH343USB0`)
- `baud_rate`: Baud rate (default: `1000000`)

**Example with custom parameters:**

```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    robot_version:=v5_5 \
    gripper_type:=100mm \
    port:=/dev/ttyUSB0 \
    baud_rate:=921600
```

### Using MoveIt in RViz

Once launched, you can use the Motion Planning plugin in RViz to:

1. **Plan motions**: Drag the interactive markers or set goal states
2. **Execute trajectories**: Click "Plan & Execute" to run on the real robot
3. **Monitor state**: View real-time joint states from the robot

### Simulation (No Real Robot)

To run MoveIt in simulation mode without hardware:

```bash
ros2 launch alicia_d_moveit demo.launch.py
```

## Configuration

### Hardware Interface Parameters

The hardware interface is configured in `alicia_d_moveit/config/alicia_d_descriptions.ros2_control.xacro`:

```xml
<hardware>
    <plugin>alicia_d_driver/AliciaDHardwareInterface</plugin>
    <param name="port">/dev/ttyCH343USB0</param>
    <param name="baud_rate">1000000</param>
    <param name="debug_mode">false</param>
    <param name="servo_count">9</param>
    <param name="gripper_type">50mm</param>
    <param name="firmware_version">auto</param>
</hardware>
```

### Controllers Configuration

Controllers are configured in `alicia_d_moveit/config/ros2_controllers.yaml`:

- **Alicia_controller**: Controls joints Joint1-Joint6
- **Gripper_controller**: Controls the right_finger joint
- **joint_state_broadcaster**: Publishes joint states

### MoveIt Controllers

MoveIt controller mapping is in `alicia_d_moveit/config/moveit_controllers.yaml`:

- Maps MoveIt planning groups to ros2_control controllers
- Uses `FollowJointTrajectory` action interface

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to serial port

**Solutions**:
1. Check cable connection
2. Verify port name: `ls /dev/tty*`
3. Check permissions: `ls -l /dev/ttyCH343USB0`
4. Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Controller Failures

**Problem**: Controllers fail to start

**Solutions**:
1. Check that the hardware interface loaded successfully:
   ```bash
   ros2 control list_hardware_interfaces
   ```
2. Verify robot is connected and powered
3. Check controller configuration:
   ```bash
   ros2 control list_controllers
   ```

### Motion Execution Issues

**Problem**: Plans succeed but execution fails

**Solutions**:
1. Verify firmware version is detected correctly (check logs)
2. Ensure gripper type matches your hardware (50mm vs 100mm)
3. Check joint limits in `joint_limits.yaml`
4. Monitor serial communication with `debug_mode:=true`

### Firmware Version

The system auto-detects firmware version. For manual configuration:

```xml
<param name="firmware_version">6.0.0</param>  <!-- Instead of "auto" -->
```

## Testing the Connection

### 1. Check Hardware Interfaces

```bash
ros2 control list_hardware_interfaces
```

Expected output should show state and command interfaces for all joints.

### 2. Check Controllers

```bash
ros2 control list_controllers
```

All controllers should be in `active` state.

### 3. Monitor Joint States

```bash
ros2 topic echo /joint_states
```

You should see real-time updates from the robot.

### 4. Test Manual Control

Send a simple trajectory:

```bash
ros2 action send_goal /Alicia_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'], 
  points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"
```

## Advanced Usage

### Programmatic Control

You can control the robot programmatically using MoveIt's Python or C++ API:

```python
#!/usr/bin/env python3
import rclpy
from moveit_py import MoveItPy

rclpy.init()
moveit = MoveItPy(node_name="moveit_py_demo")

# Get planning component
arm = moveit.get_planning_component("Alicia_arm")

# Set target pose
arm.set_goal_state(joint_names=["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"],
                   joint_values=[0.0, 0.5, 0.5, 0.0, 0.5, 0.0])

# Plan and execute
plan_result = arm.plan()
if plan_result:
    arm.execute()
```

### Zero Calibration

To calibrate the robot to zero position:

```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "{data: true}"
```

### Demonstration Mode (Zero Torque)

To enable hand-guiding mode:

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

To disable and restore full torque:

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
```

## File Structure

```
alicia_d_driver/
├── include/alicia_d_driver/
│   ├── alicia_d_hardware_interface.hpp    # Hardware interface header
│   └── alicia_d_driver_node.hpp            # Standalone driver node
├── src/
│   ├── alicia_d_hardware_interface.cpp     # Hardware interface implementation
│   ├── alicia_d_driver_node.cpp            # Standalone driver
│   └── serial_communicator.cpp             # Serial communication
├── launch/
│   └── alicia_d_driver.launch.py           # Standalone driver launch
├── alicia_d_driver.xml                     # Plugin description
└── CMakeLists.txt

alicia_d_moveit/
├── config/
│   ├── alicia_d_descriptions.ros2_control.xacro  # Hardware interface config
│   ├── ros2_controllers.yaml               # Controller configuration
│   ├── moveit_controllers.yaml             # MoveIt controller mapping
│   └── ...                                 # Other MoveIt configs
├── launch/
│   ├── real_robot.launch.py                # Real robot launch
│   ├── demo.launch.py                      # Simulation launch
│   └── ...                                 # Other launch files
└── package.xml
```

## Differences from alicia_duo_driver

The `alicia_d_driver` uses a different approach compared to `alicia_duo_driver`:

**alicia_duo_driver** (simpler):
- Standalone driver node
- Publishes to `/joint_states`
- Subscribes to custom message types
- No ros2_control integration

**alicia_d_driver** (ros2_control based):
- Hardware interface plugin for ros2_control
- Integrated with trajectory controllers
- Full MoveIt support out of the box
- Standard ROS2 Control interfaces
- Better suited for complex manipulation tasks

Both approaches work, but the ros2_control approach provides better integration with MoveIt and follows ROS2 best practices.

## Additional Resources

- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
- [ros2_control Documentation](https://control.ros.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS logs: `~/.ros/log/` or use `ros2 topic echo /rosout`
3. Enable debug mode for more verbose output
4. Check hardware connection and firmware version

