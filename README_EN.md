# Alicia-D ROS 2

[English Version](README_EN.md) | [中文版](README.md) | [Official Taobao Store](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D Product Manual (CN)](https://docs.sparklingrobo.com/)

<p align="center"><img src="./imgs/Alicia_D_v5_5.jpg" width="500" /></p>

The **Alicia-D ROS2** is a ROS2 repository for controlling the "Alicia-D" series of 6-axis robotic arms (with gripper). Built on top of the ROS2 Humble, it provides functionalities to control the arm's movement, operate the gripper, and read posture and status data via serial communication.

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

---

## ✨ Key Features

*   **ros2_control Integration**: Standard ros2_control hardware interface for seamless MoveIt2 integration.
*   **MoveIt2 Support**: Full MoveIt2 motion planning and execution capabilities.
*   **Joint Control**: Supports setting and reading the angles of six joints with smooth trajectory execution.
*   **Gripper Control**: Precise position control supporting both 50mm and 100mm gripper types.
*   **Real-time State Feedback**: Real-time retrieval of joint angles, gripper position, and robot status.
*   **Auto Firmware Detection**: Automatic firmware version detection or manual specification (supports V5 and V6 firmware).
*   **Serial Communication**: Automatic serial port detection or manual specification with high baud rate support.
*   **Zero Calibration**: Set the current position as the new zero point.
*   **Hand-guiding Mode**: Enable or disable joint motor torque for free-drag teaching.

## Project Structure

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

## Quick Start

### 1. Set Serial Port Permissions (One-time)

```bash
sudo usermod -a -G dialout $USER
```

**Then log out and log back in!**

Or temporarily:
```bash
sudo chmod 666 /dev/ttyACM0
```

### 2. Get Source Code

```bash
mkdir -p ~/alicia_ws/src
cd ~/alicia_ws
git clone https://github.com/Synria-Robotics/Alicia-D-ROS2.git -b v6.1.0 ./src
```

### 3. Install Dependencies

Install all required ROS 2 packages and system dependencies:

```bash
cd ~/alicia_ws/src
./install.sh
```

Alternatively, install dependencies manually:

```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
cd ~/alicia_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Note**: `sudo rosdep init` only needs to be run once per system. If you've already initialized rosdep, skip that step.

### 4. Build Workspace

```bash
cd ~/alicia_ws
colcon build
source install/setup.bash
```

### 5. Launch Real Robot with MoveIt

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```

**With custom parameters:**
```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=100mm \
    port:=/dev/ttyACM0 \
    speed_deg_s:=30
```

## Usage

### MoveIt Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gripper_type` | `50mm` | Gripper stroke (`50mm` or `100mm`) |
| `port` | `''` (empty string) | Serial port device path, e.g., `/dev/ttyACM0`. Leave empty for auto-detection |
| `speed_deg_s` | `20` | Default speed for joint movements (degrees per second) |

### Standalone Driver Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `''` (empty string) | Serial port device path, e.g., `/dev/ttyACM0`. Leave empty for auto-detection |
| `default_speed_deg_s` | `20.0` | Default speed for joint movements (degrees per second), range: 4.39-439.45 |

### Using MoveIt in RViz

1. **Set Goal State**: Drag interactive markers
2. **Plan Motion**: Click "Plan" button
3. **Execute Motion**: Click "Execute" or "Plan & Execute"

For detailed usage examples, see [Basic Usage Guide](docs/Basic_usage_EN.md).

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to serial port

**Solutions**:
1. Check cable connection
2. Verify port name: `ls /dev/tty*`
3. Check permissions: `ls -l /dev/ttyCH341USB0`
4. Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Controller Failures

**Problem**: Controllers fail to start

**Solutions**:
1. Check hardware interface: `ros2 control list_hardware_interfaces`
2. Check controllers: `ros2 control list_controllers`
3. Verify robot is connected and powered

### Motion Execution Issues

**Problem**: Plans succeed but execution fails

**Solutions**:
1. Verify firmware version is detected correctly (check logs)
2. Ensure gripper type matches your hardware (50mm vs 100mm)
3. Check that the `speed_deg_s` parameter is set appropriately (default: 20 deg/s)
4. Monitor serial communication with `debug_mode:=true`

## Documentation

For detailed documentation, see:

*   [Basic Usage Guide](docs/Basic_usage_EN.md) - Complete usage examples and API reference
*   [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
*   [ros2_control Documentation](https://control.ros.org/)
*   [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Safety Notes

⚠️ **Important Safety Guidelines:**

1. Always have emergency stop ready
2. Clear workspace before motion execution
3. Start with slow velocities
4. Test in simulation first
5. Monitor robot during operation

## Support

For issues or questions:
1. Check the troubleshooting section
2. Review ROS logs: `~/.ros/log/`
3. Enable debug mode: `debug_mode:=true`
4. Check hardware connection and firmware version
