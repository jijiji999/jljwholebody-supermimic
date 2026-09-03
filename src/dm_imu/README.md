# dm_imu

DM IMU 串口驱动。节点解析设备数据并发布 `sensor_msgs/msg/Imu`，供全身部署策略使用。

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

默认话题为 `/imu/data`，可在节点参数中覆盖。设备规则文件为 `config/99-dm-imu.rules`。

全身 bringup 会按需启动同一个节点：

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=false use_imu:=true
```
