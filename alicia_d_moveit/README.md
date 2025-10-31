# Alicia-D MoveIt Configuration (ROS 2)

MoveIt configuration package for Alicia-D robot with support for multiple robot versions and gripper types.

## Features

- **Multi-version support**: v5_5, v5_6
- **Multi-gripper support**: 50mm, 100mm
- **Dynamic configuration**: Select robot version and gripper type at launch time

## Supported Configurations

| Robot Version | Gripper Type | SRDF File |
|---------------|--------------|-----------|
| v5_5 | 100mm | `Alicia_D_v5_5_gripper_100mm.srdf` |
| v5_6 | 50mm | `Alicia_D_v5_6_gripper_50mm.srdf` |
| v5_6 | 100mm | `Alicia_D_v5_6_gripper_100mm.srdf` |

## Usage

### Launch MoveIt Demo

```bash
# Default: v5_6 with 50mm gripper
ros2 launch alicia_d_moveit demo.launch.py

# Specify version and gripper
ros2 launch alicia_d_moveit demo.launch.py robot_version:=v5_6 gripper_type:=100mm
ros2 launch alicia_d_moveit demo.launch.py robot_version:=v5_5 gripper_type:=100mm
```

### Launch Move Group Only

```bash
ros2 launch alicia_d_moveit move_group.launch.py robot_version:=v5_6 gripper_type:=50mm
```

### Launch RViz with MoveIt Plugin

```bash
ros2 launch alicia_d_moveit moveit_rviz.launch.py robot_version:=v5_6 gripper_type:=100mm
```

### Launch Robot State Publisher

```bash
ros2 launch alicia_d_moveit rsp.launch.py robot_version:=v5_6 gripper_type:=50mm
```

## Launch Arguments

All launch files support the following arguments:

- `robot_version`: Robot version (default: `v5_6`)
  - Valid values: `v5_5`, `v5_6`
- `gripper_type`: Gripper type (default: `50mm`)
  - Valid values: `50mm`, `100mm`

Additional arguments per launch file:

### demo.launch.py
- `db`: Start database (default: `false`)

### move_group.launch.py
- `allow_trajectory_execution`: Allow trajectory execution (default: `true`)
- `fake_execution`: Use fake execution (default: `true`)
- `info`: Print info messages (default: `true`)
- `debug`: Enable debug mode (default: `false`)

### moveit_rviz.launch.py
- `rviz_config`: RViz config file path (default: package default)

### rsp.launch.py
- `publish_frequency`: Publishing frequency for robot_state_publisher (default: `15.0`)

### warehouse_db.launch.py
- `warehouse_port`: Warehouse database port (default: `33829`)

## Package Structure

```
alicia_d_moveit/
├── config/
│   ├── Alicia_D_v5_5_gripper_100mm.srdf
│   ├── Alicia_D_v5_6_gripper_50mm.srdf
│   ├── Alicia_D_v5_6_gripper_100mm.srdf
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   └── ...
└── launch/
    ├── moveit_config_builder.py     # Custom versioned config builder
    ├── demo.launch.py
    ├── move_group.launch.py
    ├── moveit_rviz.launch.py
    ├── rsp.launch.py
    ├── static_virtual_joint_tfs.launch.py
    ├── spawn_controllers.launch.py
    ├── warehouse_db.launch.py
    └── setup_assistant.launch.py
```

## Dependencies

- `alicia_d_descriptions`: URDF description package (must have versioned URDFs)
- `moveit_ros_move_group`
- `moveit_configs_utils`
- `moveit_planners`
- Other MoveIt 2 packages (see `package.xml`)

## Notes

- URDFs are loaded from `alicia_d_descriptions/urdf/Alicia_D_{version}/Alicia_D_gripper_{type}.urdf`
- SRDFs are loaded from this package's `config/Alicia_D_{version}_gripper_{type}.srdf`
- The `moveit_config_builder.py` provides the versioning logic
- All launch files use `OpaqueFunction` to resolve version/gripper arguments at runtime

