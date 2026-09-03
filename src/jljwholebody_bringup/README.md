# jljwholebody_bringup

`jljwholebody_bringup` 是整机 ros2_control 启动包，负责把机器人描述、控制器、硬件接口、手柄和可选 IMU 组合起来。

如果要运行 `jljwholebody_deploy_node`，应先停止 bringup，避免多个入口同时控制电机。

## 1. Launch 入口

```bash
ros2 launch jljwholebody_bringup bringup.launch.py
```

默认使用 mock hardware：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py use_mock_hardware:=true
```

真实硬件：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=false \
  motor_config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_hardware/config/dm_motor_whole_body.yaml \
  enable_on_activate:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

## 2. 启动的节点

| 节点 | 来源包 | 条件 | 功能 |
| --- | --- | --- | --- |
| `robot_state_publisher` | `robot_state_publisher` | 总是启动 | 发布 robot description 和 TF |
| `ros2_control_node` | `controller_manager` | 总是启动 | 加载 `JljWholebodySystem` 或 mock hardware |
| `spawner joint_state_broadcaster` | `controller_manager` | 延迟 2 s | 发布 `/joint_states` |
| `spawner whole_body_controller` | `controller_manager` | 延迟 3 s | 启动全身 JointTrajectoryController |
| `joystick` | `jljwholebody_joystick` | `use_joystick:=true` | 发布 `/joy` |
| `dm_imu_node` | `dm_imu` | `use_imu:=true` | 发布 `/imu/data`、`/imu/status` 和可选 `/imu/pose` |

## 3. Launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_mock_hardware` | `true` | `true` 使用 `mock_components/GenericSystem`，`false` 使用 DM 电机硬件插件 |
| `motor_config_path` | 空 | 传给硬件插件；空值时硬件插件使用包内默认电机配置 |
| `parallel_mechanisms_path` | 包内 `config/parallel_mechanisms.yaml` | 传给硬件插件的并联机构配置 |
| `enable_on_activate` | `false` | ros2_control activate 时是否使能电机 |
| `disable_on_deactivate` | `true` | deactivate 时是否失能电机 |
| `command_timeout_ms` | `0` | 电机命令发送超时 |
| `feedback_poll_timeout_ms` | `0` | 电机反馈 poll 超时 |
| `lifecycle_timeout_ms` | `500` | 电机使能/失能超时 |
| `startup_feedback_warmup_s` | `0.5` | activate 使能前的反馈预热和诊断时间 |
| `enable_retries` | `3` | activate 使能失败时的重试次数 |
| `enable_retry_interval_ms` | `200` | activate 使能重试间隔 |
| `controller_manager_path` | `/controller_manager` | spawner 使用的 controller manager 路径 |
| `use_sim_time` | `false` | 仿真时间 |
| `use_joystick` | `true` | 是否启动手柄节点 |
| `joy_topic` | `/joy` | 手柄发布话题 |
| `joystick_device_path` | 环境变量 `JLJWHOLEBODY_JOY_DEVICE` 或空 | 指定 `/dev/input/event*` |
| `joystick_device_name` | 环境变量 `JLJWHOLEBODY_JOY_NAME` 或空 | 按设备名子串匹配 |
| `joystick_deadzone` | `0.10` | 摇杆死区 |
| `joystick_publish_rate` | `50.0` | 手柄发布频率 |
| `joystick_reconnect_period` | `2.0` | 手柄重连周期 |
| `joystick_frame_id` | `flydigi` | Joy header frame id |
| `use_imu` | `false` | 是否启动 IMU |
| `imu_port` | 环境变量 `JLJWHOLEBODY_IMU_PORT` 或 `/dev/dm_imu` | IMU 串口 |
| `imu_baud` | 环境变量 `JLJWHOLEBODY_IMU_BAUD` 或 `921600` | IMU 波特率 |
| `imu_frame_id` | `imu_link` | IMU frame id |
| `imu_publish_pose` | `false` | 是否发布 `/imu/pose` |

## 4. 控制器配置

控制器文件：

```text
src/jljwholebody_bringup/config/controllers.yaml
```

当前配置：

- `controller_manager.update_rate`: `1000`
- `joint_state_broadcaster`: `joint_state_broadcaster/JointStateBroadcaster`
- `whole_body_controller`: `joint_trajectory_controller/JointTrajectoryController`
- 命令接口：`position`
- 状态接口：`position`、`velocity`
- 状态发布频率：`50 Hz`

`whole_body_controller` 管理 29 个逻辑关节，顺序与部署策略 profile 一致：

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

## 5. 常用命令

查看控制器：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

查看关节状态：

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

关闭手柄：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  use_joystick:=false
```

启用 IMU：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

## 6. CAN bridge 辅助脚本

本包提供两个脚本用于 Cannelloni CAN bridge：

```bash
bash src/jljwholebody_bringup/scripts/build_cannelloni.sh
bash src/jljwholebody_bringup/scripts/start_can_bridge.sh 192.168.1.157 14000 50
```

`start_can_bridge.sh` 默认创建 `vcan0..vcan3` 并为每个接口启动一个 UDP tunnel。停止：

```bash
pkill cannelloni
```
