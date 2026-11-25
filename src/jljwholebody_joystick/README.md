# jljwholebody_joystick

`jljwholebody_joystick` 是 JLJ 全身机器人的手柄输入包。它通过 Linux `evdev`
读取飞智类手柄，并发布标准 `sensor_msgs/msg/Joy` 消息。

## 输出

- 可执行文件：`joystick_node`
- 节点默认名称：直接 `ros2 run` 时为 `jljwholebody_joystick`，随 launch 启动时为 `joystick`
- 默认话题：`/joy`
- 消息类型：`sensor_msgs/msg/Joy`
- 默认发布频率：`50 Hz`

手柄未连接时节点不会阻塞 bringup 或 deploy，而是保持运行并定期尝试重连。

## 主要代码

| 文件 | 功能 |
| --- | --- |
| `jljwholebody_joystick/joystick_node.py` | 设备发现/重连、evdev 事件归一化及 Joy 消息发布 |
| `launch/joystick.launch.py` | 手柄节点的独立 launch 和参数入口 |

## 依赖

```bash
sudo apt install python3-evdev
```

如果当前用户无法打开设备，将用户加入 `input` 组，然后注销并重新登录：

```bash
sudo usermod -aG input "$USER"
```

## 构建

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select jljwholebody_joystick
source install/setup.bash
```

## 单独运行手柄节点

自动检测第一个手柄类输入设备：

```bash
ros2 launch jljwholebody_joystick joystick.launch.py
```

指定输入设备：

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  device_path:=/dev/input/event15
```

按设备名匹配：

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  device_name_contains:=Flydigi
```

修改话题、死区或发布频率：

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  joy_topic:=/joy \
  deadzone:=0.08 \
  publish_rate:=100.0
```

## 随 Bringup 启动

主 bringup 默认启动手柄节点：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py use_mock_hardware:=true
```

关闭手柄节点：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  use_joystick:=false
```

在 bringup 中选择手柄：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  joystick_device_name:=Flydigi
```

## 随正式部署启动

deploy launch 也默认启动手柄节点：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py use_joystick:=true
```

`jljwholebody_deploy_node` uses the default mapping as follows:

| Input | Deploy function |
| --- | --- |
| `A + LB` | Switch to `passive` |
| `A + LT` | Switch to `fixed` |
| `A + RB` | Switch to `loco` |
| D-pad Up, `axes[7] = -1` | Start 50 Hz tracking recording in `loco` |
| D-pad Down, `axes[7] = +1` | Stop recording and generate reports in `loco` |
| Left stick Y, `axes[1]` | Forward/back command `lin_x` |
| Left stick X, `axes[0]` | Lateral command `lin_y` |
| Right stick X, `axes[2]` | Yaw command |

准确的索引、符号和比例位于 `src/jljwholebody_deploy/config/deploy.yaml` 的
`deployment.joystick`。

## 参数

| Parameter | Default | Description |
| --- | --- | --- |
| `joy_topic` | `/joy` | Topic used to publish `sensor_msgs/msg/Joy`. |
| `device_path` | empty | Exact Linux input path, such as `/dev/input/event15`. Empty means auto-detect. |
| `device_name_contains` | empty | Optional case-insensitive substring used to match the input device name. |
| `deadzone` | `0.10` | Deadzone for stick axes. Trigger axes use an additional small zero threshold. |
| `publish_rate` | `50.0` | Joy message publish rate in Hz. |
| `reconnect_period` | `2.0` | Seconds between reconnect attempts when no controller is connected. |
| `frame_id` | `flydigi` | Header frame id for published Joy messages. |

## 摇杆映射

| Joy field | Control |
| --- | --- |
| `axes[0]` | Left stick X |
| `axes[1]` | Left stick Y |
| `axes[2]` | Right stick X |
| `axes[3]` | Right stick Y |
| `axes[4]` | LT |
| `axes[5]` | RT |
| `axes[6]` | D-pad X |
| `axes[7]` | D-pad Y |

摇杆轴归一化到 `[-1.0, 1.0]`，LT 和 RT 归一化到 `[0.0, 1.0]`。

## 按键映射

| Joy field | Button |
| --- | --- |
| `buttons[0]` | A |
| `buttons[1]` | B |
| `buttons[2]` | X |
| `buttons[3]` | Y |
| `buttons[4]` | LB |
| `buttons[5]` | RB |
| `buttons[6]` | SELECT |
| `buttons[7]` | START |
| `buttons[8]` | L3 |
| `buttons[9]` | R3 |

`buttons[10]`～`buttons[15]` 为保留位，当前保持为 `0`。

## 检查发布数据

在另一个终端中：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joy
```

或只查看话题频率：

```bash
ros2 topic hz /joy
```

## 故障排查

列出 Linux 输入设备：

```bash
ls -l /dev/input/
ls -l /dev/input/by-id/
```

如果节点输出 `Waiting for joystick controller...`，检查手柄是否已配对、开机并显示在
`/dev/input/` 下。

如果节点无法读取设备，检查权限：

```bash
groups
ls -l /dev/input/event*
```

如果连接了多个手柄，设置 `device_path` 或 `device_name_contains`，确保机器人总是
选择预期设备。
