# jljwholebody_joystick

`jljwholebody_joystick` is the joystick input package for the JLJ whole-body
humanoid deployment workspace. It reads a Flydigi-style gamepad through Linux
`evdev` and publishes standard `sensor_msgs/msg/Joy` messages.

## Output

- Node name: `jljwholebody_joystick`
- Executable: `joystick_node`
- Default topic: `/joy`
- Message type: `sensor_msgs/msg/Joy`
- Default publish rate: `50 Hz`

The node does not block robot bringup when the controller is not connected. It
keeps running and tries to reconnect periodically.

The global robot-node index is documented in `docs/node_function_index.md`.

## Dependencies

```bash
sudo apt install python3-evdev
```

If the device cannot be opened by your user, add the user to the `input` group,
then log out and log in again:

```bash
sudo usermod -aG input "$USER"
```

## Build

From the workspace root:

```bash
./scripts/build.sh --packages-select jljwholebody_joystick
source install/setup.bash
```

## Run Only the Joystick Node

Auto-detect the first gamepad-like input device:

```bash
ros2 launch jljwholebody_joystick joystick.launch.py
```

Use a fixed input device:

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  device_path:=/dev/input/event15
```

Match a device by name:

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  device_name_contains:=Flydigi
```

Change topic, deadzone, or publish rate:

```bash
ros2 launch jljwholebody_joystick joystick.launch.py \
  joy_topic:=/joy \
  deadzone:=0.08 \
  publish_rate:=100.0
```

## Run with Robot Bringup

The main bringup starts the joystick node by default:

```bash
ros2 launch jljwholebody_bringup bringup.launch.py use_mock_hardware:=true
```

Disable the joystick node:

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  use_joystick:=false
```

Select a joystick from bringup:

```bash
ros2 launch jljwholebody_bringup bringup.launch.py \
  use_mock_hardware:=true \
  joystick_device_name:=Flydigi
```

## Run with Deployment

The deploy launch also starts the joystick node by default:

```bash
ros2 launch jljwholebody_deploy deploy.launch.py use_joystick:=true
```

`jljwholebody_deploy_node` uses the default mapping as follows:

| Input | Deploy function |
| --- | --- |
| `A + LB` | Switch to `passive` |
| `A + LT` | Switch to `fixed` |
| `A + RB` | Switch to `loco` |
| Left stick Y, `axes[1]` | Forward/back command `lin_x` |
| Left stick X, `axes[0]` | Lateral command `lin_y` |
| Right stick X, `axes[2]` | Yaw command |

The exact indices, signs, and scales live in
`src/jljwholebody_deploy/config/deploy.yaml` under `deployment.joystick`.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `joy_topic` | `/joy` | Topic used to publish `sensor_msgs/msg/Joy`. |
| `device_path` | empty | Exact Linux input path, such as `/dev/input/event15`. Empty means auto-detect. |
| `device_name_contains` | empty | Optional case-insensitive substring used to match the input device name. |
| `deadzone` | `0.10` | Deadzone for stick axes. Trigger axes use an additional small zero threshold. |
| `publish_rate` | `50.0` | Joy message publish rate in Hz. |
| `reconnect_period` | `2.0` | Seconds between reconnect attempts when no controller is connected. |
| `frame_id` | `flydigi` | Header frame id for published Joy messages. |

## Axis Mapping

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

Stick axes are normalized to `[-1.0, 1.0]`. LT and RT are normalized to
`[0.0, 1.0]`.

## Button Mapping

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

`buttons[10]` through `buttons[15]` are reserved and currently stay at `0`.

## Check the Published Data

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joy
```

Or view only the topic rate:

```bash
ros2 topic hz /joy
```

## Troubleshooting

List detected Linux input devices:

```bash
ls -l /dev/input/
ls -l /dev/input/by-id/
```

If the node logs `Waiting for joystick controller...`, check that the controller
is paired, powered on, and visible under `/dev/input/`.

If the node cannot read the device, check permissions:

```bash
groups
ls -l /dev/input/event*
```

If multiple controllers are connected, set either `device_path` or
`device_name_contains` so the robot always uses the expected device.
