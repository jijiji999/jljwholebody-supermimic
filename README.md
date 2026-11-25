# JLJ Wholebody

JLJ 全身机器人 ROS 2 Humble 工作空间。项目包含机器人模型、DM 电机通信、
`ros2_control` 调试入口、手柄和 IMU 驱动，以及直接运行 ONNX 策略的正式部署节点。

> **安全提示：** `jljwholebody_bringup` 的真实硬件模式和
> `jljwholebody_deploy` 都会向同一组电机发送命令，任何时候只能运行其中一个。
> 正式部署前应架起机器人、保留急停，并确保现场人员远离关节运动范围。

## 1. 代码结构与功能

| 路径/ROS 2 包 | 功能 | 主要入口 |
| --- | --- | --- |
| `src/jljwholebody_description` | 29 个逻辑关节的 URDF/Xacro、mesh 和 RViz 配置 | `display.launch.py` |
| `src/jljwholebody_hardware` | DM 协议、SocketCAN、电机管理、脚踝和腰部并联解算，以及 `ros2_control` 硬件插件 | `jljwholebody_hardware/JljWholebodySystem` |
| `src/jljwholebody_bringup` | 组合 robot description、`ros2_control`、控制器、手柄和可选 IMU，供模型检查和硬件调试 | `bringup.launch.py` |
| `src/jljwholebody_deploy` | 读取电机和 IMU 状态，在 C++ 中运行 ONNX 策略，并以 500 Hz 控制电机 | `deploy.launch.py` |
| `src/jljwholebody_joystick` | 通过 Linux `evdev` 读取手柄并发布 `/joy` | `joystick.launch.py` |
| `src/dm_imu` | 通过串口读取 DM IMU，发布 `/imu/data` 和 `/imu/status` | `dm_imu_node` |
| `src/serial` | `dm_imu` 使用的第三方 C++ 串口库 | 库，无机器人启动入口 |
| `third_party/cannelloni` | SocketCAN 与 LinkX UDP 之间的 CAN FD tunnel | `cannelloni` 可执行文件 |

各包的详细参数和实现说明见：

- [bringup 说明](src/jljwholebody_bringup/README.md)
- [策略部署说明](src/jljwholebody_deploy/README.md)
- [硬件层说明](src/jljwholebody_hardware/README.md)
- [机器人模型说明](src/jljwholebody_description/README.md)
- [手柄说明](src/jljwholebody_joystick/README.md)
- [IMU 说明](src/dm_imu/README.md)

## 2. 运行入口如何选择

| 目的 | 启动入口 | 是否控制真实电机 |
| --- | --- | --- |
| 只查看 URDF/RViz | `jljwholebody_description display.launch.py` | 否 |
| 检查控制器和关节接口 | `jljwholebody_bringup bringup.launch.py`（默认 mock） | 否 |
| 使用 `ros2_control` 调试真实硬件 | `jljwholebody_bringup bringup.launch.py use_mock_hardware:=false` | 是 |
| 运行训练得到的全身策略 | `jljwholebody_deploy deploy.launch.py` | 是 |

`bringup` 和 `deploy` 是两个互斥的电机控制入口。正式策略部署不需要先启动
`bringup`；部署 launch 会按参数自行启动手柄和 IMU 节点。

## 3. 构建工作空间

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每个新终端至少需要执行：

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`jljwholebody_deploy` 构建时需要 ONNX Runtime。默认从同级训练仓库
`unitree_rl_mjlab/deploy/thirdparty/` 查找，也可在构建前设置 `ONNXRUNTIME_ROOT`。

## 4. 正式策略部署

以下命令均在工作空间根目录执行。默认 LinkX 地址为 `192.168.1.157`，物理网卡为
`enp8s0`，UDP 起始端口为 `14000`。如果机器人控制机配置不同，应先修改命令参数或
设置 `ETHERNET_INTERFACE`。

### 4.1 首次构建 Cannelloni

```bash
bash src/jljwholebody_bringup/scripts/build_cannelloni.sh
```

### 4.2 启动四路 CAN bridge

```bash
bash src/jljwholebody_bringup/scripts/start_can_bridge.sh 192.168.1.157 14000 50
```

该命令检查物理网卡和 LinkX 连通性，创建 `vcan0`～`vcan3`，再分别使用 UDP
端口 `14000`～`14003` 启动 tunnel。非默认网卡示例：

```bash
ETHERNET_INTERFACE=enp3s0 \
  bash src/jljwholebody_bringup/scripts/start_can_bridge.sh 192.168.1.157 14000 50
```

确认 tunnel 已运行：

```bash
ip link show vcan0
pgrep -a cannelloni
```

### 4.3 启动前校验配置和 ONNX

该命令不会打开 CAN 或使能电机：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_deploy/config/deploy.yaml \
  validate_config_only:=true \
  use_joystick:=false \
  use_imu:=false
```

只有看到 `Configuration and native ONNX validation passed` 后再继续。

### 4.4 正式启动

先确认没有运行 `ros2_control_node` 或另一份部署节点：

```bash
pgrep -af 'ros2_control_node|jljwholebody_deploy_node'
```

确认无冲突进程后启动：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_deploy/config/deploy.yaml \
  use_joystick:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

当前 `deploy.yaml` 配置为启动时使能电机并进入 `passive`。部署模式由手柄切换：

| 手柄组合键 | 模式 | 行为 |
| --- | --- | --- |
| `A + LB` | `passive` | 低增益跟随当前反馈位置 |
| `A + LT` | `fixed` | 在配置的过渡时间内进入默认站立姿态 |
| `A + RB` | `loco` | 启用 ONNX 策略，根据摇杆速度命令控制机器人 |

`loco` 中左摇杆控制前后/横向速度，右摇杆 X 控制偏航速度。进入 `fixed` 或
`loco` 前，应先确认反馈、IMU、手柄和机器人姿态均正常。

### 4.5 运行状态与退出

另开一个已 source 工作空间的终端：

```bash
ros2 topic echo /deploy/mode
ros2 topic echo /deploy/status
ros2 topic echo /deploy/errors
ros2 topic hz /deploy/joint_states
ros2 topic hz /imu/data
ros2 topic hz /joy
```

正常退出时先按 `A + LB` 回到 `passive`，再在启动终端按 `Ctrl-C`。当前配置的
`disable_on_exit: true` 会在正常退出路径中失能电机。最后可停止 tunnel：

```bash
pkill cannelloni
```

## 5. 当前策略与配置入口

| 文件 | 用途 |
| --- | --- |
| `src/jljwholebody_deploy/config/deploy.yaml` | 控制频率、话题、安全阈值、模式增益、手柄映射和策略选择入口 |
| `src/jljwholebody_deploy/config/policies/active.yaml` | 只选择当前生效的 profile |
| `src/jljwholebody_deploy/config/policies/<run>/profile.yaml` | observation/action、关节顺序、默认姿态、限制和模型哈希 |
| `src/jljwholebody_hardware/config/dm_motor_whole_body.yaml` | 电机名称、CAN 总线、ID、型号、方向和零点 |
| `src/jljwholebody_hardware/config/parallel_mechanisms.yaml` | 脚踝、腰部逻辑关节与物理电机的并联绑定 |
| `src/jljwholebody_bringup/config/controllers.yaml` | `ros2_control` 控制器及 29 个逻辑关节顺序 |

实际生效策略始终以 `active.yaml` 的 `active_profile` 为准，不要根据目录名或旧文档
猜测。当前可用以下命令查看：

```bash
sed -n 's/^active_profile:[[:space:]]*//p' \
  src/jljwholebody_deploy/config/policies/active.yaml
```

## 6. 调试命令

只显示模型：

```bash
ros2 launch jljwholebody_description display.launch.py
```

使用 mock hardware 启动整机 `ros2_control`：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  use_joystick:=false
```

真实硬件 `ros2_control` 调试（不是正式策略部署）：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=false \
  motor_config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_hardware/config/dm_motor_whole_body.yaml \
  enable_on_activate:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

该命令退出后才能启动正式策略部署。
