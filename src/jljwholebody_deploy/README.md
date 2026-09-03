# jljwholebody_deploy

全身策略部署节点。节点在 C++ 进程内运行 ONNX Runtime，从 IMU 和 29 个逻辑关节状态构造训练任务一致的 observation，并向 29 个物理电机发送 MIT 指令。

## 策略契约

当前 profile 位于 `config/policies/jljwholebody_2026-09-01/`，由 `config/policies/active.yaml` 选择：

- observation：`490` 维，7 个 observation term，各保留 5 帧，term-major 顺序；
- action：`29` 维；
- gait period：`0.6 s`；
- 关节顺序、默认姿态、action scale、关节限制和训练执行器参数均来自 profile；
- 模型快照和 profile 保存了 SHA-256，防止训练文件继续写入导致部署漂移。

策略来源是 `/home/rx/project/sliouzhou04/unitree_rl_mjlab` 的 `jljwholebody` 任务。训练产生新模型后，应重新复制 `policy.onnx` 并更新 profile 的 `source.onnx_sha256`。

## 构建

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

CMake 会自动使用训练仓库中的 ONNX Runtime；也可以通过 `ONNXRUNTIME_ROOT` 指定其他安装路径。

## 启动

部署节点会直接打开四条 SocketCAN 并控制电机；实机策略运行时不要同时启动 `jljwholebody_bringup`（其中的 ros2_control 也会发送电机命令）。CAN bridge 可单独运行，然后直接启动策略，IMU 和手柄由这个 launch 按需启动：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_deploy/config/deploy.yaml \
  use_joystick:=true use_imu:=true
```

启动前只检查配置和 ONNX（不会打开 CAN 或电机）：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  validate_config_only:=true use_joystick:=false use_imu:=false
```

## 全身总线分配

默认配置为 `jljwholebody_hardware/config/dm_motor_whole_body.yaml`：

- `vcan0`：左臂 ID `1..7`，`waist_motor1` ID `8`，`waist_motor2` ID `9`；
- `vcan1`：右臂 ID `1..7`，物理腰 yaw 电机 `waistyaw` ID `8`；
- `vcan2`：左腿，沿用半身配置；
- `vcan3`：右腿，沿用半身配置。

逻辑关节名为 `waist_yaw`、`waist_roll`、`waist_pitch`。其中 `waist_roll`/`waist_pitch` 通过 `config/parallel_mechanisms.yaml` 使用吸收进硬件库的腰部并联解算器，`waist_yaw` 映射到物理名 `waistyaw`。

## 运行模式

节点保留 `passive`、`fixed` 和 `loco` 三种部署模式。`loco` 模式由手柄组合键切换，ONNX 推理频率和控制频率由 `deploy.yaml` 配置；并联腰部和脚踝在 loco 模式下使用逻辑关节力矩经雅可比转成两个电机力矩。

错误和状态默认发布到 `/deploy/errors`、`/deploy/status`，关节状态发布到 `/deploy/joint_states`。
