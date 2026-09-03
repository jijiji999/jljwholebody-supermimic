# jljwholebody_description

`jljwholebody_description` 保存 JLJ 全身机器人的 URDF/Xacro、mesh 和 RViz 配置，是 bringup、deploy 和模型检查的共同基础。

## 1. 文件结构

| 路径 | 说明 |
| --- | --- |
| `urdf/jljwholebody.urdf.xacro` | 机器人连杆、关节、`imu_link`、ros2_control 配置 |
| `meshes/lowbody/*.STL` | 全身 STL 模型（保留历史目录名） |
| `rviz/view_robot.rviz` | 模型检查 RViz 配置 |
| `launch/display.launch.py` | 独立模型显示入口 |

## 2. 显示模型

```bash
ros2 launch jljwholebody_description display.launch.py
```

默认启动：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

关闭关节 GUI：

```bash
ros2 launch jljwholebody_description display.launch.py use_gui:=false
```

使用真实硬件插件生成 robot description：

```bash
ros2 launch jljwholebody_description display.launch.py use_mock_hardware:=false
```

注意：`display.launch.py` 只显示模型，不启动 `ros2_control_node`，因此 `use_mock_hardware` 只影响生成的 `<ros2_control>` 插件字段。

## 3. Launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_gui` | `true` | 是否启动 `joint_state_publisher_gui` |
| `use_mock_hardware` | `true` | Xacro 中 `<ros2_control>` 是否使用 mock hardware |

## 4. 关节和 frame

主要 frame：

- `base_link`
- `waist_yaw_link`
- `waist_roll_link`
- `imu_link`
- 腰部、左右腿和左右臂的全部 link

`imu_link` 是 `base_link` 的固定子 frame，用于 `dm_imu_node` 的默认 `frame_id`。

受控逻辑关节共 29 个，顺序与策略 profile 一致：

```text
left_hip_yaw
left_hip_roll
left_hip_pitch
left_knee
left_ankle_pitch
left_ankle_roll
right_hip_yaw
right_hip_roll
right_hip_pitch
right_knee
right_ankle_pitch
right_ankle_roll
waist_yaw
waist_roll
waist_pitch
left_shoulder_pitch
left_shoulder_roll
left_shoulder_yaw
left_elbow
left_wrist_yaw
left_wrist_roll
left_wrist_pitch
right_shoulder_pitch
right_shoulder_roll
right_shoulder_yaw
right_elbow
right_wrist_yaw
right_wrist_roll
right_wrist_pitch
```

## 5. ros2_control 配置

`jljwholebody.urdf.xacro` 接收以下硬件参数，并由 bringup 传入：

| Xacro 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_mock_hardware` | `true` | 使用 mock 或真实插件 |
| `hardware_plugin` | `jljwholebody_hardware/JljWholebodySystem` | 真实硬件插件名 |
| `motor_config_path` | 空 | 电机配置路径 |
| `parallel_mechanisms_path` | 空 | 并联机构配置路径 |
| `enable_on_activate` | `false` | activate 时使能电机 |
| `disable_on_deactivate` | `true` | deactivate 时失能电机 |
| `command_timeout_ms` | `0` | 命令超时 |
| `feedback_poll_timeout_ms` | `0` | 反馈 poll 超时 |
| `lifecycle_timeout_ms` | `500` | 使能/失能超时 |
| `startup_feedback_warmup_s` | `0.5` | activate 使能前的反馈预热和诊断时间 |
| `enable_retries` | `3` | activate 使能失败时的重试次数 |
| `enable_retry_interval_ms` | `200` | activate 使能重试间隔 |

踝部逻辑关节在 ros2_control 中仍暴露为 `left/right_ankle_pitch` 和 `left/right_ankle_roll`，真实电机映射由 `jljwholebody_hardware` 的并联机构配置完成。

## 6. 修改模型时的检查

修改 URDF、mesh 或关节限制后，建议按顺序检查：

```bash
ros2 launch jljwholebody_description display.launch.py
ros2 launch jljwholebody_bringup bringup.launch.py use_mock_hardware:=true
ros2 control list_hardware_interfaces
```

如果 RViz 中模型方向正确但 IMU 驱动整机姿态不对，应优先检查 `jljwholebody_deploy` 和 `dm_imu` 文档中的 IMU 安装补偿参数。
