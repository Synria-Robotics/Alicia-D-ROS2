# alicia_d_calibration - 手眼标定模块

本模块为 Alicia-D 机械臂提供手眼标定（Eye-in-Hand）功能，基于 ArUco 标记和相机实现末端执行器到相机的空间变换。

> [!warning]
> 本模块以 Intel Realsense D405 和 Orbbec Gemini 335 相机为例。如使用其它相机，可能需要修改相关文件。

## 📋 模块说明

**alicia_d_calibration** 是一个完整的手眼标定解决方案，用于：
- 计算机械臂末端与相机之间的空间关系
- 输出标定结果转换矩阵
- 支持各种 ArUco 标记配置
- 提供标定结果验证功能

## 📁 文件结构

```
alicia_d_calibration/
├── scripts/
│   ├── hand_eye_calibration.py             # 手眼标定主脚本
│   ├── calibration_verifier.py             # 标定验证脚本
│   └── aruco_detector.py                   # ArUco 标记检测器
├── launch/
│   ├── hand_eye_calibration.launch.py      # 标定启动文件
│   └── verify_calibration.launch.py        # 标定验证启动文件
├── config/
│   └── hand_eye_calibration_result.yaml    # 标定结果输出文件
├── package.xml                             
├── CMakeLists.txt                          
└── README.md                          
```

## 🚀 快速开始

### 1. 环境准备

在运行标定前，确保需要的环境已经设置：

```bash
# 创建 conda 环境（如果需要）
conda create -n calib python=3.10 -y
conda activate calib
conda install pip -y

# 安装依赖
pip install "numpy<2.0.0" "opencv-python<4.11" scipy pyyaml jinja2 typeguard
```

> [!note]
> 若非明确指定，请尽量确保退出 ``conda`` 再运行程序 ：
> ```bash
> conda deactivate
> ```


### 2. 启动必要的节点

在不同的终端中依次启动：

**终端 1：启动机械臂驱动和 MoveIt**

> ``gripper type`` 根据实际情况修改（50/100mm）

```bash
ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm
```

**终端 2：启动相机驱动**

若使用Realsense D405:

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    infra_rgb:=true \
    pointcloud.enable:=true
```

若使用Gemini 335:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    enable_left_ir:=true \
    enable_right_ir:=true \
    enable_point_cloud:=true \
    enable_colored_point_cloud:=true
```

### 3. 运行标定

**终端 3：执行标定启动文件**  

 **3.1 眼在手内（Eye-in-Hand）**
 

>**准备 ArUco 标记**：标记固定在工作台或墙面上，保持不动。建议距离相机 25-45cm，标记平面尽量正对相机视角，避免强反光。


**终端：激活环境**

```bash
conda activate calib
```

**启动标定（默认 Realsense D405）：**

```bash
ros2 launch alicia_d_calibration hand_eye_calibration.launch.py \
    calibration_type:=eye_in_hand
```

**若使用 Gemini 335 相机，请添加参数：**

```bash
ros2 launch alicia_d_calibration hand_eye_calibration.launch.py \
    calibration_type:=eye_in_hand \
    camera_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info
```

**3.2 眼在手外（Eye-to-Hand）**

>眼在手外外参标定环境要求高，同时对于机械臂末端执行器的安装要求也较高，建议零基础用户优先尝试眼在手内标定。标定完成后不能移动相机位置和机械臂基座位置，否则需要重新标定。

**准备 ArUco 标记**：标记固定在末端执行器上（夹具或夹爪），确保安装牢固，不晃动。

<p align="center"><img src="../imgs/eye_to_hand_example.png" width="400" height="500" /></p>

**终端：激活环境**

```bash
conda activate calib
```

**启动标定（默认 Realsense D405）：**

```bash
ros2 launch alicia_d_calibration hand_eye_calibration.launch.py \
    calibration_type:=eye_to_hand
```

**若末端偏高（标记偏离相机视野），可调整 Joint2/Joint3 偏移量：**

```bash
ros2 launch alicia_d_calibration hand_eye_calibration.launch.py \
    calibration_type:=eye_to_hand \
    eye_to_hand_joint2_offset:=-0.04 \
    eye_to_hand_joint3_offset:=-0.06
```


**若使用 Gemini 335 相机，请添加参数：**

```bash
ros2 launch alicia_d_calibration hand_eye_calibration.launch.py \
    calibration_type:=eye_to_hand \
    camera_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info
```

标定结束后，标定结果会自动保存在 `alicia_d_calibration/config/hand_eye_calibration_result.yaml`

**3.3 标定注意事项（强烈建议阅读）**

- 相机内参必须正确，否则标定结果会系统性偏差。
- 标记必须清晰可见，避免遮挡、强反光。
- 采集过程中机械臂不要抖动或碰撞标记。
- 眼在手外时，标记固定在末端，确保安装刚性、无松动。
- 眼在手内时，标记固定在环境，确保标记不移动。
- 若标定不稳定，优先检查光照、标记尺寸、相机曝光和采样范围。

<p align="center"><img src="../imgs/eye_in_hand_calib.png" width="500" /></p>

### 4. 验证标定结果

> 请先 ``CTRL+C`` 终止标定启动文件

**终端 3：执行标定验证文件**

若使用Realsense D405相机：
```bash
ros2 launch alicia_d_calibration verify_calibration.launch.py
```
若使用Gemini 335相机，请添加参数：
```bash
ros2 launch alicia_d_calibration verify_calibration.launch.py \
    camera_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info
```

**终端 4：验证标定**

查看 TF 树：
```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

验证标定结果:
```bash
ros2 run tf2_ros tf2_echo base_link aruco_marker_frame
```

若标定类型为 `eye_to_hand`（眼在手外，标记固定在末端），请改为：
```bash
ros2 run tf2_ros tf2_echo gripper_center aruco_marker_frame
```

> [!note]
> 验证脚本会输出稳定性指标（`std_t` / `std_r`），若平移标准差在 2cm 以上或旋转标准差在 2° 以上，
> 通常意味着数据质量不佳（光照、遮挡、标记抖动或采样不足）。



## ✅ 标定流程

1. **准备阶段**：确保相机标定参数正确，打印或显示 ArUco 标记
2. **采集阶段**：脚本自动移动机械臂，在 20+ 个不同位置采集数据
3. **计算阶段**：利用采集的数据计算手眼变换矩阵
4. **保存阶段**：标定结果保存为 YAML 文件
5. **验证阶段**：通过验证脚本测试标定精度
