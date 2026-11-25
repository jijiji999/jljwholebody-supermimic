# jljwholebody_deploy

全身策略正式部署包。节点直接打开四条 SocketCAN，在 C++ 进程内运行 ONNX
Runtime，从 IMU 和 29 个逻辑关节状态构造与训练任务一致的 observation，并向
物理电机发送 MIT 指令。

> **安全提示：** 本节点和真实硬件 `jljwholebody_bringup` 都会控制同一组电机，
> 不能同时运行。完整的现场操作顺序见仓库根目录
> [正式策略部署](../../README.md#4-正式策略部署)。

## 主要代码

| 路径 | 功能 |
| --- | --- |
| `src/jljwholebody_deploy_node.cpp` | 配置加载、状态采集、500 Hz 控制循环、50 Hz 策略循环、模式切换和安全检查 |
| `src/wholebody_policy.cpp` | ONNX Runtime 会话、模型输入输出和推理 |
| `launch/deploy.launch.py` | 组合部署节点、手柄节点和可选 IMU 节点 |
| `config/deploy.yaml` | 运行频率、话题、安全阈值、控制增益、手柄和策略入口 |
| `config/policies/active.yaml` | 当前生效 profile 的选择器 |
| `scripts/prepare_policy_deployment.py` | 从训练 run 生成、校验、复制并可选激活策略 bundle |
| `scripts/generate_policy_profile.py` | 从训练配置、ONNX metadata 和 MJCF 生成策略契约 |
| `scripts/check_policy_contract.py` | 校验 ONNX、env、MJCF、profile 及 URDF 契约 |

## 策略契约

当前 profile 由 `config/policies/active.yaml` 动态选择。实际值请查看：

```bash
sed -n 's/^active_profile:[[:space:]]*//p' \
  src/jljwholebody_deploy/config/policies/active.yaml
```

profile 定义以下策略契约：

- observation：`490` 维，7 个 observation term，各保留 5 帧，term-major 顺序；
- action：`29` 维；
- gait period：`0.6 s`；
- 关节顺序、默认姿态、action scale、关节限制和训练执行器参数均来自 profile；
- 模型快照和 profile 保存了 SHA-256，防止训练文件继续写入导致部署漂移。

策略来源是 `/home/rx/project/sliouzhou04/unitree_rl_mjlab` 的 `jljwholebody` 任务。
不要只替换 ONNX 文件；新策略应通过 `prepare_policy_deployment.py` 生成不可变 bundle、
校验哈希并更新 `active.yaml`。

## 构建

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

CMake 会自动使用训练仓库中的 ONNX Runtime；也可以通过 `ONNXRUNTIME_ROOT` 指定其他安装路径。

## 启动

CAN bridge 单独启动后，直接运行部署 launch。IMU 和手柄由该 launch 按需启动：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_deploy/config/deploy.yaml \
  use_joystick:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

启动前只检查配置和 ONNX（不会打开 CAN 或电机）：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  config_path:=/home/rx/project/sliouzhou04/jljwholebody/src/jljwholebody_deploy/config/deploy.yaml \
  validate_config_only:=true \
  use_joystick:=false \
  use_imu:=false
```

校验成功时会打印 `Configuration and native ONNX validation passed`，并正常退出，
不会打开 CAN 或使能电机。

## 全身总线分配

默认配置为 `jljwholebody_hardware/config/dm_motor_whole_body.yaml`：

- `vcan0`：左臂 ID `1..7`，`waist_motor1` ID `8`，`waist_motor2` ID `9`；
- `vcan1`：右臂 ID `1..7`，物理腰 yaw 电机 `waistyaw` ID `8`；
- `vcan2`：左腿，沿用半身配置；
- `vcan3`：右腿，沿用半身配置。

逻辑关节名为 `waist_yaw`、`waist_roll`、`waist_pitch`。其中 `waist_roll`/`waist_pitch` 通过 `config/parallel_mechanisms.yaml` 使用吸收进硬件库的腰部并联解算器，`waist_yaw` 映射到物理名 `waistyaw`。

## 运行模式

节点提供 `passive`、`fixed`、`loco` 和 `danc` 四种部署模式。当前配置启动时使能电机并
进入 `passive`：

| 组合键 | 模式 | 说明 |
| --- | --- | --- |
| `A + LB` | `passive` | 低增益跟随反馈位置；启动和退出前的安全模式 |
| `A + LT` | `fixed` | 平滑过渡到 profile 默认姿态 |
| `A + RB` | `loco` | 运行 ONNX 策略；要求手柄、IMU 和电机反馈均有效 |
| `RB + X` | `danc` | 等 `fixed` 站姿过渡结束后触发 SuperMimic；动作播完回到 `fixed` |
| 十字键上 | `loco` 内开始记录 | 以 50 Hz 记录逻辑关节期望值与实际值 |
| 十字键下 | `loco` 内停止记录 | 异步写入 CSV、摘要和离线 HTML 报告 |

ONNX 推理频率和控制频率由 `deploy.yaml` 配置。并联腰部和脚踝在 `loco` 模式下
使用逻辑关节力矩经雅可比转换为两个电机的力矩。

主要运行话题：

| 话题 | 说明 |
| --- | --- |
| `/deploy/mode` | 当前 `passive`、`fixed`、`loco` 或 `danc` 模式 |
| `/deploy/status` | 控制/策略循环和输入状态 |
| `/deploy/errors` | 部署故障与安全检查错误 |
| `/deploy/joint_states` | 29 个逻辑关节状态 |
| `/deploy/projected_gravity` | 用于策略的投影重力 |
| `/deploy/base_ang_vel` | 用于策略的角速度 |

正常退出时先用 `A + LB` 返回 `passive`，再按 `Ctrl-C`。当前配置的
`disable_on_exit: true` 会在正常退出路径中失能电机。

## SuperMimic 模型与离线验证

本仓库已包含 `deploy.yaml` 默认引用的 SuperMimic 策略包。模型、外部权重和
参考轨迹随配置一起安装。从工作空间根目录构建并执行离线验证。

```bash
colcon build --packages-select jljwholebody_deploy
source install/setup.bash
ros2 run jljwholebody_deploy jljwholebody_deploy_node --ros-args -p validate_config_only:=true
```

需要从上游重新生成策略包时，使用带 Git LFS 的 `jlj_robot` 工作副本和本项目的
`scripts/prepare_supermimic.py`。用 `--output` 指定新的输出目录，再通过
`deployment.danc.bundle_path` 选择该目录。

在日志中确认出现 `Native SuperMimic ready`，表示模型、参考轨迹和关节映射已加载且
首帧推理成功。实际运行时，`/deploy/status` 中的 `danc_frame` 显示播放进度。

当前参考轨迹是 `walk2_subject3`。上游 MuJoCo 回放从参考首帧初始化，前 500 步通过
高度检查，完整回放在 51.62 秒跌倒。从本项目 `fixed` 站姿接入 C++ 推理的 MuJoCo
回放在 51.44 秒超过 50° 倾斜阈值。默认 `deployment.danc.shadow_mode: true` 让
`danc` 执行推理并保持 `fixed` 姿态。[上游模型和仿真脚本](https://github.com/Vicky-Yu007/jlj_robot/tree/b75c2b6a770e5d2ac1769b2c2dd7b4123cded3f4)

实机力矩输出需要在 `deployment.danc.joint_calibration_path` 指向标定 YAML。顶层
用 29 个逻辑关节名作键，每个关节填写 `direction`（`1` 或 `-1`）和
`offset_rad`，满足 `实机关节角 = offset_rad + direction × 策略关节角`。
两套模型的上肢关节轴在各自零位存在差异。完成实机关节方向、零位和并联机构核对，
并通过要执行的参考动作闭环仿真后，再使用 `shadow_mode: false`。

## Loco 数据记录

进入 `loco` 后，按一次十字键上开始记录，按一次十字键下停止并生成报告。若记录中
退出 `loco` 或节点退出，当前记录也会安全停止并写盘。默认输出目录为启动命令当前
目录下的 `log/jljwholebody_deploy/<时间戳_run_N>/`。

每次记录包含逻辑关节的期望/实际位置、速度和力矩（或直线关节力），并联腰部和脚踝
按虚拟 pitch/roll 关节记录，不展开为两个电机。打开 `index.html` 可选择指标、关节 A、
可选的对比关节 B，以及显示期望或实际曲线；`joint_timeseries.csv` 保留完整原始数据。
按键轴、阈值、采样率和输出路径可在 `config/deploy.yaml` 的
`deployment.joystick.recording` 与 `deployment.recording` 中配置。

## 切换训练策略

以下命令会生成 profile、复制 ONNX/env/MJCF、执行契约校验，并仅在全部成功后更新
`active.yaml`：

```bash
python3 src/jljwholebody_deploy/scripts/prepare_policy_deployment.py \
  --run-dir /home/rx/project/sliouzhou04/unitree_rl_mjlab/logs/<task>/<run> \
  --activate
```

激活后先重新执行只读校验，再重启部署节点。不要在节点运行中切换文件。
