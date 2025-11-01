# Alicia-D ROS 2


[English Version](README_EN.md) | [中文版](README.md) | [官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D 产品手册（中文）](https://docs.sparklingrobo.com/)

<p align="center"><img src="./imgs/Alicia_D_v5_5.jpg" width="500" /></p>



**Alicia-D ROS2** 是一个用于控制【灵动 Alicia-D】系列六轴机械臂（带夹爪）的 ROS 工具包。它基于 ROS 2 Humble 构建，提供通过串口通信控制机械臂运动、操作夹爪、读取姿态与状态数据等功能。

## 目录

- [快速开始](#快速开始)
- [安装与设置](#安装与设置)
- [使用方法](#使用方法)
- [配置](#配置)
- [故障排除](#故障排除)
- [系统架构](#系统架构)
- [高级用法](#高级用法)

## 快速开始

### 前置要求

- 已安装 ROS 2 Humble
- Alicia-D 机械臂通过 USB 连接
- 已配置串口权限

### 1. 设置串口权限（一次性）

```bash
sudo usermod -a -G dialout $USER
```

**然后需要完全注销并重新登录！**

或临时设置：
```bash
sudo chmod 666 /dev/ttyCH341USB0
```

### 2. 编译工作空间

```bash
cd ~/alicia_ws
colcon build --packages-select alicia_d_driver alicia_d_moveit
source install/setup.bash
```

### 3. 启动真实机械臂与 MoveIt

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```

**使用自定义参数：**
```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    robot_version:=v5_6 \
    gripper_type:=100mm \
    port:=/dev/ttyCH341USB0 \
    baud_rate:=1000000 \
    firmware_version:=auto
```

## 安装与设置

### 系统依赖

```bash
sudo apt update
sudo apt install libserial-dev python3-numpy
sudo apt-get install -y ros-humble-asio-cmake-module ros-humble-io-context
```

### 获取源代码

```bash
mkdir -p ~/alicia_ws/src
cd ~/alicia_ws
git clone https://github.com/Synria-Robotics/Alicia-D-ROS2.git -b v6.0.0 ./src
```

### 编译工作空间

```bash
cd ~/alicia_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### 配置环境

```bash
echo "source ~/alicia_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 验证串口连接

```bash
ls -l /dev/ttyCH341USB0
# 或
ls -l /dev/ttyUSB*
```

## 使用方法

### 启动选项

#### 选项 1：真实机械臂与 MoveIt（推荐）

```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    robot_version:=v5_6 \
    gripper_type:=50mm \
    port:=/dev/ttyCH341USB0 \
    baud_rate:=1000000 \
    firmware_version:=auto
```

#### 选项 2：仅仿真（无硬件）

```bash
ros2 launch alicia_d_moveit demo.launch.py \
    robot_version:=v5_6 \
    gripper_type:=50mm
```

#### 选项 3：独立驱动（无 MoveIt）

```bash
ros2 launch alicia_d_driver alicia_d_driver.launch.py
```

### Launch 文件参数

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `robot_version` | `v5_6` | 机械臂版本 (`v5_5` 或 `v5_6`) |
| `gripper_type` | `50mm` | 夹爪行程 (`50mm` 或 `100mm`) |
| `port` | `/dev/ttyUSB0` | 串口设备 |
| `baud_rate` | `1000000` | 串口波特率 |
| `firmware_version` | `auto` | 固件版本 (`5.0.0`, `6.0.0`, 或 `auto` 自动检测) |

### 在 RViz 中使用 MoveIt

1. **设置目标状态**
   - 拖动交互式标记点
   - 或使用 "Planning" 选项卡 → "Select Start State" / "Select Goal State"

2. **规划运动**
   - 点击 "Plan" 按钮
   - 在可视化中查看轨迹

3. **执行运动**
   - 点击 "Execute" 或 "Plan & Execute"
   - 机械臂将跟随规划的轨迹运动

## 配置

### 硬件接口参数

硬件接口在 `alicia_d_moveit/config/alicia_d_descriptions.ros2_control.xacro` 中配置：

```xml
<hardware>
    <plugin>alicia_d_driver/AliciaDHardwareInterface</plugin>
    <param name="port">/dev/ttyCH341USB0</param>
    <param name="baud_rate">1000000</param>
    <param name="debug_mode">false</param>
    <param name="servo_count">9</param>
    <param name="gripper_type">50mm</param>
    <param name="firmware_version">auto</param>
</hardware>
```

### 控制器配置

控制器在 `alicia_d_moveit/config/ros2_controllers.yaml` 中配置：

- **Alicia_controller**: 控制关节 Joint1-Joint6
- **Gripper_controller**: 控制 right_finger 关节
- **joint_state_broadcaster**: 发布关节状态

### MoveIt 控制器

MoveIt 控制器映射在 `alicia_d_moveit/config/moveit_controllers.yaml` 中：

- 将 MoveIt 规划组映射到 ros2_control 控制器
- 使用 `FollowJointTrajectory` Action 接口

## 验证命令

### 监控关节状态

```bash
ros2 topic echo /joint_states
```



## 特殊命令


运行 `alicia_d_driver.launch.py`：
```bash
ros2 launch alicia_d_driver alicia_d_driver.launch.py \
    port:=/dev/ttyCH343USB0 \
    gripper_type:=100mm \
    firmware_version:=6.0.0
```

### 使能手引导模式（零力矩）

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

### 禁用手引导（恢复全力矩）

```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: false}"
```

### 零位校准

步骤 1：禁用力矩
```bash
ros2 topic pub --once /demonstration std_msgs/msg/Bool "{data: true}"
```

步骤 2：将机械臂放置到期望的姿势

步骤 3：执行校准
```bash
ros2 topic pub --once /zero_calibrate std_msgs/msg/Bool "{data: true}"
```

## 故障排除

### 连接问题

**问题**：无法连接到串口

**解决方案**：
1. 检查线缆连接
2. 验证端口名称：`ls /dev/tty*`
3. 检查权限：`ls -l /dev/ttyCH341USB0`
4. 将用户添加到 dialout 组：`sudo usermod -a -G dialout $USER`
5. 尝试不同的端口：`port:=/dev/ttyUSB0`

### 控制器故障

**问题**：控制器启动失败

**解决方案**：
1. 检查硬件接口是否成功加载：
   ```bash
   ros2 control list_hardware_interfaces
   ```
2. 验证机械臂已连接并通电
3. 检查控制器配置：
   ```bash
   ros2 control list_controllers
   ```

### 运动执行问题

**问题**：规划成功但执行失败

**解决方案**：
1. 验证固件版本是否正确检测（查看日志）
2. 确保夹爪类型与硬件匹配（50mm vs 100mm）
3. 检查 `joint_limits.yaml` 中的关节限制
4. 使用 `debug_mode:=true` 监控串口通信
5. 尝试显式指定固件版本：`firmware_version:=5.0.0`

### 固件版本

系统支持自动检测固件版本。手动配置：

```bash
ros2 launch alicia_d_moveit real_robot.launch.py firmware_version:=5.0.0
```

或在 xacro 文件中：
```xml
<param name="firmware_version">6.0.0</param>  <!-- 替代 "auto" -->
```

## 系统架构

```
MoveIt → Joint Trajectory Controllers → ros2_control → Hardware Interface → Serial → Robot
                                                ↓
                                        Joint State Publisher
                                                ↓
                                           /joint_states
```

### 关键组件

1. **硬件接口** (`alicia_d_hardware_interface.cpp`)
   - 实现 ros2_control `SystemInterface`
   - 处理与机械臂的串口通信
   - 在 ROS 关节状态和硬件协议之间转换
   - 支持固件版本检测和多种夹爪类型

2. **ROS2 Control 配置**
   - **Controller Manager**: 管理所有控制器
   - **Joint State Broadcaster**: 发布关节状态到 `/joint_states`
   - **Alicia_controller**: 6自由度机械臂的关节轨迹控制器
   - **Gripper_controller**: 夹爪的关节轨迹控制器

3. **Launch 文件**
   - **real_robot.launch.py**: 用于通过 MoveIt 控制真实机械臂的启动文件
   - **demo.launch.py**: 带有假执行的原始演示（仿真）



## 文件结构

```
alicia_d_driver/
├── include/alicia_d_driver/
│   ├── alicia_d_hardware_interface.hpp    # 硬件接口头文件
│   └── alicia_d_driver_node.hpp            # 独立驱动节点
├── src/
│   ├── alicia_d_hardware_interface.cpp     # 硬件接口实现
│   ├── alicia_d_driver_node.cpp            # 独立驱动
│   └── serial_communicator.cpp             # 串口通信
├── launch/
│   └── alicia_d_driver.launch.py           # 独立驱动启动
├── alicia_d_driver.xml                     # 插件描述
└── CMakeLists.txt

alicia_d_moveit/
├── config/
│   ├── alicia_d_descriptions.ros2_control.xacro  # 硬件接口配置
│   ├── ros2_controllers.yaml               # 控制器配置
│   ├── moveit_controllers.yaml             # MoveIt 控制器映射
│   └── ...                                 # 其他 MoveIt 配置
├── launch/
│   ├── real_robot.launch.py                # 真实机械臂启动
│   ├── demo.launch.py                      # 仿真启动
│   └── ...                                 # 其他启动文件
└── package.xml
```

## 安全注意事项

⚠️ **重要的安全指南：**

1. 始终准备好紧急停止
2. 在执行运动前清理工作区域
3. 从慢速开始
4. 首先在仿真中测试
5. 在操作过程中监控机械臂
6. 检查关节限制是否正确配置

## 其他资源

- [MoveIt 2 文档](https://moveit.picknik.ai/main/index.html)
- [ros2_control 文档](https://control.ros.org/)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

## 支持

如有问题或疑问：
1. 查看上述故障排除部分
2. 查看 ROS 日志：`~/.ros/log/` 或使用 `ros2 topic echo /rosout`
3. 启用调试模式以获取更详细的输出
4. 检查硬件连接和固件版本

