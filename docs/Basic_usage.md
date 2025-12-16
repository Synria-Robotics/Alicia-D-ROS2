# Alicia-D ROS2 使用指南

本文档提供 Alicia-D ROS2 的完整使用说明和示例。




# 仿真

对于 `50mm` 夹爪类型，使用默认值：

```
ros2 launch alicia_d_moveit demo.launch.py
```

对于 `100mm` 夹爪类型，指定夹爪类型：

```
ros2 launch alicia_d_moveit demo.launch.py gripper_type:=100mm
```



# Real Robot MoveIt

基本启动（使用默认参数）：
```
ros2 launch alicia_d_moveit real_robot.launch.py
```

指定夹爪类型：
```
ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm
```

指定串口（自动检测时可不指定）：
```
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=100mm \
    port:=/dev/ttyACM0
```

指定运动速度（度/秒）：
```
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=50mm \
    port:=/dev/ttyACM0 \
    speed_deg_s:=30
```

## Launch 参数说明

### MoveIt Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gripper_type` | `50mm` | 夹爪类型：`50mm` 或 `100mm` |
| `port` | `''` (空字符串) | 串口设备路径，如 `/dev/ttyACM0`。留空则自动检测 |
| `speed_deg_s` | `20` | 关节运动的默认速度（度/秒） |

# 独立驱动节点（无 MoveIt）

独立驱动节点提供基础的机械臂控制功能，不依赖 MoveIt。

## 启动独立驱动

基本启动（使用默认参数）：
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py
```

指定串口：
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py port:=/dev/ttyACM0
```

指定默认速度：
```
ros2 launch alicia_d_driver alicia_d_driver.launch.py \
    port:=/dev/ttyACM0 \
    default_speed_deg_s:=30.0
```

## 独立驱动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `''` (空字符串) | 串口设备路径，如 `/dev/ttyACM0`。留空则自动检测 |
| `default_speed_deg_s` | `20.0` | 关节运动的默认速度（度/秒），范围：4.39-439.45 |

## 独立驱动 Topics

### 订阅 Topics

- `/joint_commands` (sensor_msgs/JointState) - 发送关节位置和速度命令
- `/demonstration` (std_msgs/Bool) - 使能/禁用示教模式（零力矩）
- `/zero_calibrate` (std_msgs/Bool) - 执行零位校准

### 发布 Topics

- `/joint_states` (sensor_msgs/JointState) - 发布当前关节状态（位置、速度）

## 使用示例

### 使能示教模式（零力矩）

使能示教模式后，可以手动拖动机械臂进行示教：

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

### 禁用示教模式（恢复全力矩）

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
```

### 零位校准
**⚠️ 注意：此操作不可逆，如非必要请忽略此步骤。**
步骤 1：先禁用力矩（进入示教模式）
```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

步骤 2：将机械臂手动移动到期望的零位姿势

步骤 3：执行零位校准（校准后会自动恢复力矩）


```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "{data: true}"
```

### 发送关节命令

通过 `/joint_commands` topic 发送关节位置和速度命令：

**示例 1：移动到零位，使用默认速度（20 度/秒）**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0]
"
```

**示例 2：指定速度移动（30 度/秒 = 0.524 弧度/秒）**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [0.785, 0.785, 0.0, 0.0, 0.0, 0.0, 500.0]
velocity: [0.524, 0.524, 0.524, 0.524, 0.524, 0.524, 0.0]
"
```

**示例 3：高速移动（100 度/秒 = 1.745 弧度/秒）**
```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "
name: ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Gripper']
position: [1.57, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [1.745, 1.745, 1.745, 1.745, 1.745, 1.745, 0.0]
"
```

**单位说明：**

`/joint_commands` topic 使用 `sensor_msgs/JointState` 消息类型，遵循 ROS 标准约定：
- 关节位置单位为**弧度（rad）**（ROS 标准）
- `velocity` 字段中的速度单位为**弧度/秒（rad/s）**（ROS 标准）

**重要提示：** 虽然驱动内部使用度/秒（`speed_deg_s`），但 ROS 消息标准要求使用 rad/s。驱动会自动将 rad/s 转换为 deg/s，因此必须按照 ROS 约定提供 rad/s 的速度值。

**行为说明：**
- 如果提供了 rad/s 的速度值，驱动会将其转换为 deg/s，并使用最大值作为所有关节的公共速度
- 如果不提供速度，将使用 `default_speed_deg_s` 参数值（单位为度/秒）
- 夹爪位置范围为 0-1000（0 为完全打开，1000 为完全闭合）

**速度转换参考（用于 /joint_commands topic）：**
- 20 度/秒 = 0.349 弧度/秒 → 使用 `velocity: [0.349, 0.349, ...]`
- 30 度/秒 = 0.524 弧度/秒 → 使用 `velocity: [0.524, 0.524, ...]`
- 40 度/秒 = 0.698 弧度/秒 → 使用 `velocity: [0.698, 0.698, ...]`
- 100 度/秒 = 1.745 弧度/秒 → 使用 `velocity: [1.745, 1.745, ...]`

**快速转换公式：** `弧度/秒 = 度/秒 × π / 180` 或 `弧度/秒 = 度/秒 × 0.0174533`


