# jljwholebody_hardware

提供全身 DM 电机通信和 `ros2_control` 硬件插件。硬件库同时处理直驱关节、左右脚踝并联机构，以及腰部 pitch/roll 并联机构。

## 主要组件

- `jljwholebody_hardware/JljWholebodySystem`：ros2_control `SystemInterface` 插件；
- `MotorManager`、`DmMotorDriver`、`SocketCanTransport`：DM 协议和 SocketCAN；
- `ParallelKinematics`：脚踝解算器和吸收进本包的腰部并联解算器。

本 workspace 不构建半身调试/频率测试节点；实机部署只使用硬件插件和 `jljwholebody_deploy_node`，避免多个程序同时控制同一条 CAN 总线。

## 配置

默认电机配置：`config/dm_motor_whole_body.yaml`。

- `vcan0`：左臂 ID `1..7`，`waist_motor1` ID `8`，`waist_motor2` ID `9`；
- `vcan1`：右臂 ID `1..7`，`waistyaw` ID `8`；
- `vcan2`：左腿，沿用半身的 ID 和 CAN；
- `vcan3`：右腿，沿用半身的 ID 和 CAN。

并联绑定在 `config/parallel_mechanisms.yaml`：

```yaml
- name: waist
  pitch_joint: waist_pitch
  roll_joint: waist_roll
  motor_1: waist_motor1
  motor_2: waist_motor2
  config_path: waist/waist.yaml
```

`config/waist/waist.yaml` 的几何、方向和雅可比实现来自 `/home/rx/project/sliouzhou04/waist_parallel_solver`，已编译进本包，不依赖外部 Python 或 ROS 节点。

## ros2_control

机器人描述为 `jljwholebody_description/urdf/jljwholebody.urdf.xacro`，包含 29 个逻辑 revolute joints。直接关节按 `motor_name` 映射；`waist_yaw` 的物理电机名为 `waistyaw`；脚踝和腰部 pitch/roll 通过并联绑定进行正逆解和速度/力矩映射。

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=false \
  enable_on_activate:=false \
  use_imu:=true
```

硬件插件参数包括 `motor_config_path`、`parallel_mechanisms_path`、`enable_on_activate`、`disable_on_deactivate`、反馈/命令超时和启动重试参数。留空时使用包内全身配置。

## CAN bridge

`jljwholebody_bringup/scripts/start_can_bridge.sh` 默认创建 `vcan0..vcan3` 并启动 Cannelloni tunnel：

```bash
bash src/jljwholebody_bringup/scripts/start_can_bridge.sh 192.168.1.157 14000 50
```
