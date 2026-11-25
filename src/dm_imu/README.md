# dm_imu

DM IMU 串口驱动。节点配置并解析设备数据，发布 `sensor_msgs/msg/Imu`，供全身
部署策略使用。底层串口访问由工作空间内的 `serial` 包提供。

## 主要代码

| 文件 | 功能 |
| --- | --- |
| `src/imu_driver.cpp` | ROS 2 参数、串口配置和消息发布 |
| `src/imu_stream_parser.cpp` | IMU 字节流分帧与字段解析 |
| `src/bsp_crc.cpp` | 设备协议 CRC 校验 |
| `src/test_imu.cpp` | `dm_imu_node` 的进程入口 |

## 构建

```bash
cd /home/rx/project/sliouzhou04/jljwholebody
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dm_imu
source install/setup.bash
```

## 运行

```bash
ros2 run dm_imu dm_imu_node --ros-args \
  -p port:=/dev/dm_imu \
  -p baud:=921600 \
  -p frame_id:=imu_link
```

默认发布 `/imu/data` 和 `/imu/status`；设置 `publish_pose:=true` 后还会发布
`/imu/pose`。设备规则文件为 `config/99-dm-imu.rules`。

全身 bringup 会按需启动同一个节点：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=false use_imu:=true
```

正式策略部署同样会启动该节点，默认已经启用 IMU：

```bash
ros2 launch jljwholebody_deploy deploy.launch.py \
  use_joystick:=true \
  use_imu:=true \
  imu_port:=/dev/dm_imu
```

不要在 bringup/deploy 已启动 IMU 时再单独运行第二个 `dm_imu_node`，否则两个进程会
争用同一个串口。
