# Alicia-D ROS 2


[English Version](README_EN.md) | [中文版](README.md) | [官方淘宝店](https://g84gtpygdv6trpvdhcsy0kfr73avcip.taobao.com/shop/view_shop.htm?appUid=RAzN8HWKU5B7MfX6JjEWgkuNfftNVbnrjbjx6fPjY9KqXB46Rvy&spm=a21n57.1.hoverItem.2) | [Alicia-D 产品手册（中文）](https://docs.sparklingrobo.com/)

<p align="center"><img src="./imgs/Alicia_D_v5_5.jpg" width="500" /></p>



**Alicia-D ROS2** 是一个用于控制【灵动 Alicia-D】系列六轴机械臂（带夹爪）的 ROS 工具包。它基于 ROS 2 Humble 构建，提供通过串口通信控制机械臂运动、操作夹爪、读取姿态与状态数据等功能。

[![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

---

> [!note] 
> 详细说明请访问[官网文档](https://docs.sparklingrobo.com/docs/alicia-d-series/)

## ✨ 主要特性

*   **ros2_control 集成**：基于标准的 ros2_control 硬件接口，支持与 MoveIt2 无缝集成。
*   **MoveIt2 支持**：完整的 MoveIt2 运动规划与执行功能。
*   **关节控制**：支持设置与读取六个关节的角度，提供平滑轨迹执行。
*   **夹爪控制**：支持精确位置控制，适配 50mm 和 100mm 两种夹爪类型。
*   **实时状态反馈**：实时获取关节角、夹爪位置与机器人状态。
*   **自动固件检测**：自动检测固件版本或手动指定（支持 V5 和 V6 固件）。
*   **串口通信**：自动搜索串口或手动指定，支持高波特率通信。
*   **零位校准**：将当前位置设置为新的零点。
*   **扭矩开关**：开启或关闭关节电机扭矩，实现自由拖动示教。

## 项目结构

```
alicia_d_driver/
├── include/alicia_d_driver/
│   ├── alicia_d_hardware_interface.hpp    # 硬件接口头文件
│   └── alicia_d_driver_node.hpp          # 独立驱动节点
├── src/
│   ├── alicia_d_hardware_interface.cpp   # 硬件接口实现
│   ├── alicia_d_driver_node.cpp          # 独立驱动
│   └── serial_communicator.cpp           # 串口通信
├── launch/
│   └── alicia_d_driver.launch.py         # 独立驱动启动
├── alicia_d_driver.xml                   # 插件描述
└── CMakeLists.txt

alicia_d_moveit/
├── config/
│   ├── alicia_d_descriptions.ros2_control.xacro  # 硬件接口配置
│   ├── ros2_controllers.yaml                     # 控制器配置
│   ├── moveit_controllers.yaml                   # MoveIt 控制器映射
│   └── ...                                       # 其他 MoveIt 配置
├── launch/
│   ├── real_robot.launch.py                      # 真实机械臂启动
│   ├── demo.launch.py                            # 仿真启动
│   └── ...                                       # 其他启动文件
└── package.xml

alicia_d_calibration/
├── scripts/
│   ├── hand_eye_calibration.py                   # 手眼标定主脚本
│   ├── generate_calibration_poses.py             # 生成标定位置序列
│   └── aruco_detector.py                         # ArUco 标记检测器
├── launch/
│   ├── hand_eye_calibration.launch.py            # 标定启动文件
│   └── verify_calibration.launch.py              # 标定验证启动文件
├── config/
│   └── hand_eye_calibration_result.yaml          # 标定结果输出文件
├── package.xml
├── CMakeLists.txt
└── README.md

alicia_d_cube_sort/
├── scripts/
│   ├── cube_detection.py                        # 立方体检测节点
│   └── cube_sorting.py                          # 分拣控制节点
├── launch/
│   ├── cube_detection.launch.py                 # 检测启动文件
│   └── cube_sorting.launch.py                   # 分拣启动文件
├── config/
│   └── cube_sorting.yaml                        # 配置文件
├── alicia_d_cube_sort/
│   ├── __init__.py                              # Python 包初始化
│   └── utils/                                   # 辅助工具模块
├── package.xml
├── CMakeLists.txt
└── README.md

alicia_d_grasp_6d/
├── scripts/
│   ├── intel_rs_d405                            # Realsense D405 相关脚本
│   └──  orbbec_gemini_335                       # Orbbec Gemini 335 相关脚本
└── README.md
```

## 快速开始

### 1. 设置串口权限

**方法1：临时设置串口权限**

```bash
sudo chmod 666 /dev/ttyACM*
```

**方法2：添加用户到dialout组**（永久有效）

```bash
sudo usermod -a -G dialout $USER
```
> 注意：需要注销（Log out）再登陆，或者重启使权限生效。

### 2. 获取源代码

```bash
mkdir -p ~/alicia_ws/src
cd ~/alicia_ws
git clone https://github.com/Synria-Robotics/Alicia-D-ROS2.git -b v6.1.0 ./src
```

### 3. 安装依赖

安装所有必需的 ROS 2 包和系统依赖：

```bash
cd ~/alicia_ws/src
./install.sh
```

或者手动安装依赖：

```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
cd ~/alicia_ws
rosdep install --from-paths src --ignore-src -r -y
```

**注意**：`sudo rosdep init` 每个系统只需运行一次。如果已经初始化过 rosdep，请跳过该步骤。

### 4. 编译工作空间

```bash
cd ~/alicia_ws
colcon build
source install/setup.bash
```

### 5. 启动真实机械臂

#### 使用 MoveIt（推荐）

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```

**自定义参数：**
```bash
ros2 launch alicia_d_moveit real_robot.launch.py \
    gripper_type:=100mm \
    port:=/dev/ttyACM0 \
    speed_deg_s:=30
```

#### 独立驱动（无 MoveIt）

```bash
ros2 launch alicia_d_driver alicia_d_driver.launch.py
```


## 使用方法

### MoveIt Launch 文件参数

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `gripper_type` | `50mm` | 夹爪行程 (`50mm` 或 `100mm`) |
| `port` | `''` (空字符串) | 串口设备路径，如 `/dev/ttyACM0`。留空则自动检测 |
| `speed_deg_s` | `20` | 关节运动的默认速度（度/秒） |

### 独立驱动 Launch 文件参数

| 参数 | 默认值 | 说明 |
|-----------|---------|-------------|
| `port` | `''` (空字符串) | 串口设备路径，如 `/dev/ttyACM0`。留空则自动检测 |
| `default_speed_deg_s` | `20.0` | 关节运动的默认速度（度/秒），范围：4.39-439.45 |

详细使用示例请参阅 [使用指南](docs/Basic_usage.md)。

## 故障排除

### 连接问题

**问题**：无法连接到串口

**解决方案**：
1. 检查线缆连接
2. 验证端口名称：`ls /dev/tty*`
3. 检查权限：`ls -l /dev/ttyACM0`
4. 将用户添加到 dialout 组：`sudo usermod -a -G dialout $USER`

### 控制器故障

**问题**：控制器启动失败

**解决方案**：
1. 检查硬件接口：`ros2 control list_hardware_interfaces`
2. 检查控制器：`ros2 control list_controllers`
3. 验证机械臂已连接并通电

### 运动执行问题

**问题**：规划成功但执行失败

**解决方案**：
1. 验证固件版本是否正确检测（查看日志）
2. 确保夹爪类型与硬件匹配（50mm vs 100mm）
3. 使用 `debug_mode:=true` 监控串口通信

## 文档

详细文档请参阅：

*   [使用指南](docs/Basic_usage.md) - 完整的使用示例和 API 参考
*   [MoveIt 2 文档](https://moveit.picknik.ai/main/index.html)
*   [ros2_control 文档](https://control.ros.org/)
*   [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

## 安全注意事项

⚠️ **重要的安全指南：**

1. 始终准备好紧急停止
2. 在执行运动前清理工作区域
3. 从慢速开始
4. 首先在仿真中测试
5. 在操作过程中监控机械臂

## 支持

如有问题或疑问：
1. 查看故障排除部分
2. 查看 ROS 日志：`~/.ros/log/`
3. 启用调试模式：`debug_mode:=true`
4. 检查硬件连接和固件版本
