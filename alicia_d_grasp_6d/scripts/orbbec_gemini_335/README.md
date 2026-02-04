# Orbbec Gemini 335 6D 抓取

基于 Orbbec Gemini 335 双目相机的 6D 抓取位姿生成与执行系统。

## 配置

### 获取源码

在 ``alicia_d_grasp_6d/`` 目录下：
```bash
git clone https://github.com/NVlabs/GraspGen.git
git clone https://github.com/NVlabs/FoundationStereo.git
mkdir SAM2
cd SAM2
git clone https://github.com/facebookresearch/sam2.git
cd ..
```
相应地，您需要创建三个conda虚拟环境，建议命名 ``graspgen``, ``foundation_stereo``, ``sam2``。虚拟环境的创建和配置须参考官方文档：

[GraspGen](https://github.com/NVlabs/GraspGen)

[FoundationStereo](https://github.com/NVlabs/FoundationStereo)

[SAM2](https://github.com/facebookresearch/sam2)

您需要根据官方文档进行配置，建议使用教程中的"pip installation"而非docker。您需要根据文档下载官方权重，并跑通文档中推理部分的demo。


## 启动顺序

### 1. 启动机械臂和相机

```bash
ros2 launch alicia_d_moveit real_robot.launch.py
```
相机需打开左右目红外：
```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_left_ir:=true enable_right_ir:=true
```

### 2. 启动 ROS 桥接节点

```bash
# 系统 Python 环境
python gemini_ros_bridge.py
```

### 3. 启动 MeshCat 可视化服务器

```bash
# graspgen 环境
conda activate graspgen
meshcat-server
```
在浏览器中打开输出连接。

### 4. 启动抓取执行节点

```bash
# 系统 Python 环境（退出conda）
python gemini_execution.py
```

**交互操作**:
- `y`：执行当前抓取
- `n`：跳过，查看下一个
- `q`：退出

### 5. 启动深度估计节点（FoundationStereo）

```bash
# FoundationStereo 环境
conda activate foundation_stereo
python gemini_foundationstereo.py --visualize
```

**参数**:
- `--ckpt_dir`: 模型权重路径
- `--scale`: 图像缩放比例（默认 1.0）
- `--z_far`: 最大深度（默认 3.0m）
- `--denoise_cloud`: 启用点云去噪（默认开启）

> 该节点推理过程较长，需稍适等待。输出深度图后，按q退出图片，继续进行以下操作。

> 该节点非实时推理，如相机视野场景改变，需重新运行。

<p align="center"><img src="../../../imgs/Gemini335_FoundationStereo.png" width="500" /></p>

### 6. 启动目标分割节点（SAM2）

```bash
# SAM2 环境
conda activate sam2
python gemini_sam2.py
```

**参数**:
- `--model`: 模型大小 [tiny/small/base/large]（默认 large）
- `--bridge_port`: ZeroMQ 端口（默认 5557）

**交互操作**:
- 左键点击：添加正样本点（目标区域）
- 右键点击：添加负样本点（背景区域）
- `r` 键：重置选择
- `Enter`：确认分割
- `q`：退出

<p align="center"><img src="../../../imgs/Gemini335_sam2.png" width="500" /></p>

### 7. 启动抓取生成节点（GraspGen）

```bash
# GraspGen 环境
conda activate graspgen
python gemini_graspgen.py
```

**参数**:
- `--gripper_config`: 夹爪配置文件
- `--grasp_threshold`: 置信度阈值（默认 0.8）
- `--num_grasps`: 生成抓取数量（默认 200）
- `--topk_num_grasps`: 返回 top-k 抓取（默认 100）

<p align="center"><img src="../../../imgs/Gemini335_GraspGen.png" width="500" /></p>

## 文件说明

| 文件 | 功能 |
|------|------|
| `gemini_ros_bridge.py` | ROS 2 与 ZeroMQ 桥接，转发相机图像 |
| `gemini_foundationstereo.py` | FoundationStereo 深度估计，生成点云 |
| `gemini_sam2.py` | SAM2 交互式目标分割 |
| `gemini_graspgen.py` | GraspGen 抓取位姿生成 |
| `gemini_execution.py` | MoveIt 2 抓取执行 |
| `utils/transform_utils.py` | 坐标变换工具函数 |

## 数据目录

- `.bridge_data/`: 节点间数据交换目录
- `outputs/`: 输出文件（点云、深度图等）
