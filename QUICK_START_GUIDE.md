# Quick Start Guide - Alicia-D with MoveIt2

## Prerequisites

- ROS 2 Humble installed
- Alicia-D robot connected via USB
- Serial port permissions configured

## Setup

### 1. Set Serial Port Permissions (One-time)

```bash
sudo usermod -a -G dialout $USER
```

**Then log out and log back in!**

Or temporarily:
```bash
sudo chmod 666 /dev/ttyCH343USB0
```

### 2. Build the Workspace

```bash
cd ~/alicia_ws
colcon build --packages-select alicia_d_driver alicia_d_moveit
source install/setup.bash
```

## Launch Options

### Option 1: Real Robot with MoveIt (Recommended)

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```

**With parameters:**
```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    robot_version:=v5_6 \
    gripper_type:=50mm \
    port:=/dev/ttyCH343USB0 \
    baud_rate:=1000000
```

### Option 2: Simulation Only (No Hardware)

```bash
ros2 launch alicia_d_moveit demo.launch.py \
    robot_version:=v5_6 \
    gripper_type:=50mm
```

### Option 3: Standalone Driver (No MoveIt)

```bash
ros2 launch alicia_d_driver alicia_d_driver.launch.py
```

## Verification Commands

### Check Hardware Interface Status

```bash
# List hardware components
ros2 control list_hardware_components

# List all interfaces
ros2 control list_hardware_interfaces

# Check controller status
ros2 control list_controllers
```

### Monitor Joint States

```bash
ros2 topic echo /joint_states
```

### Send Test Commands

```bash
# Test arm controller
ros2 action send_goal /Alicia_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'], 
  points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"
```

## Using MoveIt in RViz

1. **Set Goal State**
   - Drag interactive markers
   - Or use "Planning" tab → "Select Start State" / "Select Goal State"

2. **Plan Motion**
   - Click "Plan" button
   - Review trajectory in visualization

3. **Execute on Robot**
   - Click "Execute" button
   - Robot will follow the planned trajectory

## Special Commands

### Zero Calibration

```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "{data: true}"
```

### Enable Hand-Guiding Mode (Zero Torque)

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

### Disable Hand-Guiding (Restore Torque)

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
```

## Launch File Parameters

### real_robot.launch.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_version` | `v5_6` | Robot version (`v5_5` or `v5_6`) |
| `gripper_type` | `50mm` | Gripper stroke (`50mm` or `100mm`) |
| `port` | `/dev/ttyCH343USB0` | Serial port device |
| `baud_rate` | `1000000` | Serial baud rate |

### demo.launch.py (Simulation)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_version` | `v5_6` | Robot version |
| `gripper_type` | `50mm` | Gripper type |
| `db` | `false` | Enable warehouse database |

## Troubleshooting

### Problem: Cannot connect to serial port

**Solutions:**
1. Check cable: `ls -l /dev/tty*`
2. Check permissions: `groups` (should include `dialout`)
3. Try different port: `port:=/dev/ttyUSB0`

### Problem: Controllers fail to spawn

**Solutions:**
1. Check hardware connection
2. Verify hardware interface loaded:
   ```bash
   ros2 control list_hardware_components
   ```
3. Check logs for errors
4. Try lower baud rate: `baud_rate:=921600`

### Problem: Motion execution fails

**Solutions:**
1. Check joint limits in `config/joint_limits.yaml`
2. Verify gripper type matches hardware
3. Check firmware version detection in logs
4. Enable debug mode in xacro files

### Problem: RViz shows wrong robot model

**Solutions:**
1. Check robot_version parameter
2. Check gripper_type parameter
3. Restart RViz
4. Clear RViz config: `rm ~/.rviz2/default.rviz`

## File Locations

```
/home/ubuntu/alicia_ws/
├── install/setup.bash              ← Source this!
└── src/
    ├── alicia_d_driver/            ← Hardware interface
    ├── alicia_d_moveit/            ← MoveIt configuration
    ├── alicia_d_descriptions/      ← Robot URDF
    ├── REAL_ROBOT_SETUP.md         ← Detailed guide
    ├── ROS2_CONTROL_INTEGRATION_SUMMARY.md  ← Technical details
    └── QUICK_START_GUIDE.md        ← This file
```

## Example Workflow

### 1. Basic Test

```bash
# Terminal 1: Launch system
ros2 launch alicia_d_moveit real_robot.launch.py

# Terminal 2: Monitor states
ros2 topic echo /joint_states

# Terminal 3: Check controllers
ros2 control list_controllers
```

### 2. Test Motion Planning

```bash
# Launch with robot
ros2 launch alicia_d_moveit real_robot.launch.py

# In RViz:
# 1. Drag end-effector to desired pose
# 2. Click "Plan & Execute"
# 3. Watch robot move!
```

### 3. Programmatic Control (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rclpy.init()
    node = rclpy.create_node('test_motion')
    
    # Create action client
    client = ActionClient(node, FollowJointTrajectory, 
                          '/Alicia_controller/follow_joint_trajectory')
    client.wait_for_server()
    
    # Create trajectory
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 
                                    'Joint4', 'Joint5', 'Joint6']
    
    # Add waypoint
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.5, 0.5, 0.0, 0.5, 0.0]
    point.time_from_start.sec = 3
    goal.trajectory.points.append(point)
    
    # Send goal
    client.send_goal_async(goal)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Next Steps

- ✅ Test with real hardware
- ✅ Calibrate kinematics if needed
- ✅ Adjust joint limits in `joint_limits.yaml`
- ✅ Create custom motion plans
- ✅ Integrate with perception (cameras, etc.)

## Support

- **Documentation**: See `REAL_ROBOT_SETUP.md` for detailed information
- **Technical Details**: See `ROS2_CONTROL_INTEGRATION_SUMMARY.md`
- **ROS2 Control**: https://control.ros.org/
- **MoveIt2**: https://moveit.picknik.ai/

## Safety Notes

⚠️ **Important Safety Guidelines:**

1. Always have emergency stop ready
2. Clear workspace before motion execution
3. Start with slow velocities
4. Test in simulation first
5. Monitor robot during operation
6. Check joint limits are properly configured

---

**Ready to use!** Start with simulation (demo.launch.py) to familiarize yourself with the system, then move to real robot control (real_robot.launch.py).

