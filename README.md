
# 机械臂控制系统 v1.2 (ROS 2 Humble 移植版)

## 1. 产品概述

机械臂控制系统 v1.2 是一套基于 ROS 2 Humble 的完整解决方案，专为6自由度机械臂及夹爪设计。系统提供标准化的 ROS 2 接口，使用弧度作为统一角度单位，简化了机械臂的控制过程，屏蔽底层通信细节，让用户专注于应用开发。

**核心功能：**

*   6自由度机械臂实时控制
*   全关节状态实时反馈 (发布 `/arm_joint_state`)
*   标准 ROS 2 接口
*   串口自动重连机制
*   通过 Launch 文件统一启动和配置

## 2. 系统架构

系统由四个主要节点组成，它们都在 `alicia_d_driver` 包中，共同构成完整的控制链路：

| 节点名称 (Launch 中) | 可执行文件                     | 语言 | 功能描述                                                                 |
| :------------------- | :------------------------------- | :--- | :----------------------------------------------------------------------- |
| `serial_driver`      | `alicia_d_driver_node`       | C++  | 负责与硬件串口通信，处理原始数据帧的收发（订阅 `/send_serial_data`，发布 `/read_serial_data`）。 |
| `serial_dispatcher`  | `serial_dispatcher_node.py`    | Py   | 订阅 `/read_serial_data`，根据命令码将原始数据帧分类转发至对应处理节点的话题（如 `/servo_states`, `/gripper_angle`, `/error_frame_deal`）。 |
| `joint_state_publisher` | `joint_state_publisher_node.py`| Py   | 订阅 `/servo_states` 和 `/gripper_angle`，处理硬件状态数据，转换为标准关节状态（弧度）并发布到 `/arm_joint_state`。 |
| `arm_control`        | `arm_control_node.py`          | Py   | 订阅标准关节命令 (`/arm_joint_command`)、单独夹爪命令 (`/gripper_control`) 及其他控制命令，转换为硬件协议格式并发布到 `/send_serial_data`。 |



## 3. 接口规范

### 3.1 主要用户接口

用户主要通过以下话题与系统交互：

| 话题名称             | 消息类型                           | 方向        | 描述                                                                |
| :------------------- | :--------------------------------- | :---------- | :------------------------------------------------------------------ |
| `/joint_states`   | `sensor_msgs/msg/JointState`  | **读取/发送**    | 订阅/发布此话题以获取/控制机械臂所有关节和夹爪    |
| `/zero_calibrate`    | `std_msgs/msg/Bool`                | **发送**    | 发布 `true` 到此话题以触发机械臂的零位校准命令。                   |
| `/demonstration`     | `std_msgs/msg/Bool`                | **发送**    | 发布 `true` 使能，发布 `false`失能。 |

### 3.2 内部接口 (调试用)

| 话题名称             | 消息类型                           | 方向                      | 描述                                           |
| :------------------- | :--------------------------------- | :------------------------ | :--------------------------------------------- |
| `/send_serial_data`  | `std_msgs/msg/UInt8MultiArray`     | `arm_control` -> `serial_driver` | 控制节点生成的发送给硬件的原始字节帧。         |
| `/read_serial_data`  | `std_msgs/msg/UInt8MultiArray`     | `serial_driver` -> `serial_dispatcher` | C++驱动从硬件读取并验证通过的原始字节帧。      |
| `/servo_states`      | `std_msgs/msg/UInt8MultiArray`     | `serial_dispatcher` -> `joint_state_publisher` | 分发出的原始舵机状态帧 (CMD 0x04)。            |                |
| `/servo_states_main` | `std_msgs/msg/Float32MultiArray`   | `joint_state_publisher` -> | 兼容 ROS 1 的关节状态数组发布（弧度）。      |



### 3.4 数据单位与限制

所有用户接口的角度值使用**弧度**作为标准单位，以下用度为单位仅为更直观阐述关节范围。


| 接口名称       | 关节序号  | 物理位置   | 运动范围 (度)   |
| :------------- | :------ | :--------- | :-------------- |
| `Joint1`       | 关节 0   | 底座旋转   | -180° 到 +180°  |
| `Joint2`       | 关节 1   | 肩部关节   | -90° 到 +90°  |
| `Joint3`       | 关节 2   | 肘部关节   | -45° 到 +135°  |
| `Joint4`       | 关节 3   | 腕部旋转   | -180° 到 +180°  |
| `Joint5`       | 关节 4   | 腕部俯仰   | -90° 到 +90°  |
| `Joint6`       | 关节 5   | 腕部翻转   | -180° 到 +180°  |
| `gripper`      | 夹爪     | 末端执行器 | 0° 到 100°      |

**注意：** 夹爪的角度范围受限于 0-100 度。

## 4. 系统部署与安装

### 4.1 系统依赖安装

*   **ROS 2 Humble:** 确保已安装 ROS 2 Humble Desktop Full。
*   **系统库:** 安装 `libserial-dev` 用于 C++ 串口通信。
    ```bash
    sudo apt update
    sudo apt install libserial-dev
    ```
*   **Python 库:** 安装 `numpy`。
    ```bash
    sudo apt install python3-numpy # 或者 pip3 install numpy
    ```
    (`rclpy` 和 `std_msgs` Python 库通常随 ROS 2 Desktop 一起安装)。
* 
```
sudo apt-get install -y ros-humble-asio-cmake-module ros-humble-io-context
```

### 4.2 获取源代码

1.  **克隆源代码:** 
    ```bash
    mkdir -p ~/alicia_ws/src
    cd ~/alicia_ws
    git clone https://github.com/Synria-Robotics/Alicia-D-ROS2.git ./src
    ```

### 4.3 编译工作空间

```bash
cd ~/alicia_ws
rosdep install --from-paths src --ignore-src -r -y 
colcon build
```


### 4.4 配置环境变量

将工作空间的 `setup` 文件添加到你的 shell 配置文件中，以便每次打开新终端时自动加载：

```bash
echo "source ~/alicia_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
# 如果你使用 zsh，请使用 setup.zsh 和 .zshrc
```

### 4.5 设置串口权限

为了让 ROS 节点能够访问串口设备 (如 `/dev/ttyUSB0`)，需要设置权限：

*   **方法1：添加用户到 `dialout` 组 (推荐，永久有效):**
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    **重要:** 执行此命令后，你需要**完全注销当前用户并重新登录**才能使组更改生效。

*   **方法2：创建 udev 规则 (推荐，永久有效):**
    创建一个文件 `/etc/udev/rules.d/99-serial.rules` 并添加以下内容（如果文件已存在，请编辑）：
    ```
    KERNEL=="ttyUSB*", MODE="0666"
    KERNEL=="ttyACM*", MODE="0666"
    ```
    然后重新加载规则：
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    这会在设备插入时自动设置权限。

*   **方法3：临时设置权限 (每次重启或重插后都需要):**
    ```bash
    sudo chmod 666 /dev/ttyUSB0 # 将 ttyUSB0 替换为你的设备
    ```



### 4.7 硬件连接验证

1.  将机械臂通过 USB 连接至计算机。
2.  检查串口是否被系统识别：
    ```bash
    ls -l /dev/ttyUSB* /dev/ttyUSB*
    ```
    记下你的设备名称 (例如 `ttyUSB0`)。




## 5. 系统启动

使用提供的 Launch 文件启动所有节点：

```bash
ros2 launch alicia_d_driver alicia_d_driver.launch.py [参数名称:=参数值 ...]
```

rviz only:
```
ros2 launch alicia_d_descriptions display.launch.py robot_version:=v5_6 gripper_type:=100mm
```



## 6. Moveit 使用方法

```
ros2 launch alicia_duo_moveit demo.launch.py 
```
在`alicia_d_driver.launch.py`运行的同时，运行上述指令可以通过`moveit`控制机械臂


## 7. 故障排除

### 7.1 常见问题

| 问题描述         | 可能原因                                         | 解决方法                                                                                             |
| :--------------- | :----------------------------------------------- | :--------------------------------------------------------------------------------------------------- |
| Launch 启动失败  | 脚本未找到、权限不足、依赖缺失                 | 检查 CMakeLists.txt 安装指令、`chmod +x`、`colcon build` 是否成功、`package.xml` 依赖是否完整             |
| 无法连接到串口   | 权限不足、端口名称错误、设备未连接               | 检查串口权限设置 (`dialout` 组或 udev 规则)、确认 `port` 参数、检查物理连接和 `ls /dev/tty*`           |
| 通信中断/不稳定 | USB 连接问题、串口线质量                          | 使用高质量 USB 线缆、尝试不同 USB 端口、检查连接是否松动                                                   |
| 硬件无响应       | 波特率错误、校验和错误、协议不匹配、硬件状态问题 | 确认 `baudrate` 参数、**仔细核对硬件协议文档修正校验和及转换逻辑**、检查硬件供电和状态、使用 `py_debug` 查看发送的帧 |
| 运动范围/方向错误| 角度转换错误、关节映射错误                       | **仔细核对硬件协议文档修正 `rad_to_hardware_value` / `_grip` 函数**、检查 `joint_to_servo_map`             |
| Checksum 失败   | 校验和算法/范围不匹配、通信干扰                 | **首要任务：根据硬件文档修正校验和计算!** (Python 控制节点 + C++ 驱动节点都需要)、检查硬件连接                 |

## 8. 注意事项

1.  **安全第一:** 确保机械臂工作区域安全，无障碍物或人员。
2.  **角度限制:** 命令应在有效范围内，尤其注意夹爪的 0-100 度限制。
3.  **数据单位:** 用户接口使用**弧度 (radians)**。
4.  **校验和与转换:** **务必根据你的具体硬件协议文档，仔细验证并修改 Python 控制节点 (`arm_control_node.py`) 和 C++ 驱动节点 (`serial_server_node.cpp`) 中的校验和计算 (`calculate_checksum`/`sumElements`)、角度到硬件值的转换 (`rad_to_hardware_value`/`_grip`) 以及关节到舵机的映射 (`joint_to_servo_map`)。这是确保系统正常工作的关键。**
5.  **启动顺序:** 使用 launch 文件可确保节点按合理顺序（或并行）启动。
6.  **串口权限:** 首次使用或更换 USB 端口可能需要重新配置权限。
7.  **停止:** 使用 `Ctrl+C` 在运行 launch 文件的终端中安全地停止所有节点。

## 9. 附录

### 9.1 系统依赖

*   ROS 2 Humble Hawksbill (Desktop Full 推荐)
*   `libserial-dev` (系统库, 通过 `apt` 安装)
*   `python3-numpy` (Python 库, 通过 `apt` 或 `pip` 安装)


### 9.2 术语表

| 术语       | 说明                                                     |
| :--------- | :------------------------------------------------------- |
| 关节 (Joint) | 机械臂的可运动部分 (0-5 对应 joint1-6)                     |
| 夹爪 (Gripper)| 末端执行器                                               |
| 弧度 (Radian)| 角度的标准单位 (π 弧度 = 180 度)                         |
| 话题 (Topic) | ROS 2 中节点间传递消息的通道                             |
| 节点 (Node)  | ROS 2 系统中的一个独立运行的程序                         |
| 发布 (Publish) | 向话题发送消息的操作                                     |
| 订阅 (Subscribe)| 从话题接收消息的操作                                     |
| Launch 文件 | 用于配置和启动一个或多个 ROS 2 节点的脚本 (通常是 Python) |

### 9.3 重要参数参考

| 参数         | 值                | 说明                                                         |
| :----------- | :---------------- | :----------------------------------------------------------- |
| 默认波特率   | 921600            | 可通过 launch 参数 `baudrate` 修改                           |
| 关节角度限制 | -π 到 +π          | 控制节点会截断超出此范围的输入                               |
| 夹爪角度限制 | 0 到 ~1.745 rad  | 约 0° 到 100°，控制节点会截断                                  |
| 舵机值范围   | 0 - 4095          | 对应 -180° 到 +180° (根据 `rad_to_hardware_value` 函数)      |
| 夹爪值范围   | 2048 - 2900       | 对应 0° 到 100° (根据 `rad_to_hardware_value_grip` 函数)     |
| 默认串口     | `ttyUSB0`         | 可通过 launch 参数 `port` 修改                               |
| 通信协议帧   | AA CMD LEN ... CHK FF | 基本帧结构，具体内容和校验和**需参照硬件文档**              |

