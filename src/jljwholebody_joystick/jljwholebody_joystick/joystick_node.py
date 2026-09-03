#!/usr/bin/env python3

from __future__ import annotations

from threading import Lock, Thread
from typing import Dict, Iterable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from evdev import InputDevice, ecodes, list_devices


class JljWholebodyJoystickNode(Node):
    """Read a Flydigi-style gamepad with evdev and publish sensor_msgs/Joy."""

    def __init__(self) -> None:
        super().__init__("jljwholebody_joystick")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("device_path", "")
        self.declare_parameter("device_name_contains", "")
        self.declare_parameter("deadzone", 0.10)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("reconnect_period", 2.0)
        self.declare_parameter("frame_id", "flydigi")

        self.joy_topic = (
            self.get_parameter("joy_topic").get_parameter_value().string_value
        )
        self.device_path = (
            self.get_parameter("device_path").get_parameter_value().string_value
        )
        self.device_name_contains = (
            self.get_parameter("device_name_contains")
            .get_parameter_value()
            .string_value
        )
        self.deadzone = (
            self.get_parameter("deadzone").get_parameter_value().double_value
        )
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        reconnect_period = (
            self.get_parameter("reconnect_period").get_parameter_value().double_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than 0")
        if reconnect_period <= 0.0:
            raise ValueError("reconnect_period must be greater than 0")

        self.publisher = self.create_publisher(Joy, self.joy_topic, 10)

        self.axis_map = {
            ecodes.ABS_X: 0,
            ecodes.ABS_Y: 1,
            ecodes.ABS_RX: 2,
            ecodes.ABS_RY: 3,
            ecodes.ABS_Z: 4,
            ecodes.ABS_RZ: 5,
            ecodes.ABS_HAT0X: 6,
            ecodes.ABS_HAT0Y: 7,
        }
        self.axis_name = {
            0: "Left_X",
            1: "Left_Y",
            2: "Right_X",
            3: "Right_Y",
            4: "LT",
            5: "RT",
            6: "DPad_X",
            7: "DPad_Y",
        }
        self.button_map = {
            ecodes.BTN_A: 0,
            ecodes.BTN_B: 1,
            ecodes.BTN_X: 2,
            ecodes.BTN_Y: 3,
            ecodes.BTN_TL: 4,
            ecodes.BTN_TR: 5,
            ecodes.BTN_SELECT: 6,
            ecodes.BTN_START: 7,
            ecodes.BTN_THUMBL: 8,
            ecodes.BTN_THUMBR: 9,
        }
        self.button_name = {
            0: "A",
            1: "B",
            2: "X",
            3: "Y",
            4: "LB",
            5: "RB",
            6: "SELECT",
            7: "START",
            8: "L3",
            9: "R3",
        }

        self.axis_center: Dict[int, float] = {}
        self.axis_min: Dict[int, float] = {}
        self.axis_max: Dict[int, float] = {}
        self.device: Optional[InputDevice] = None
        self.reader_thread: Optional[Thread] = None
        self.lock = Lock()

        self.msg = Joy()
        self.msg.header.frame_id = self.frame_id
        self.msg.axes = [0.0] * 8
        self.msg.buttons = [0] * 16

        self.create_timer(1.0 / publish_rate, self.publish_joy)
        self.create_timer(reconnect_period, self.ensure_connected)

        self.print_mapping()
        self.ensure_connected()

    def ensure_connected(self) -> None:
        if self.device is not None:
            return

        device = self.find_device()
        if device is None:
            self.get_logger().warn("Waiting for joystick controller...")
            return

        self.device = device
        self.init_axis(device)
        self.get_logger().info(f"Connected joystick: {device.name} ({device.path})")

        self.reader_thread = Thread(target=self.read_events, daemon=True)
        self.reader_thread.start()

    def find_device(self) -> Optional[InputDevice]:
        paths = [self.device_path] if self.device_path else list_devices()

        for path in paths:
            if not path:
                continue
            try:
                device = InputDevice(path)
                if not self.is_supported_device(device):
                    device.close()
                    continue
                return device
            except OSError as exc:
                self.get_logger().debug(f"Cannot open joystick device {path}: {exc}")

        return None

    def is_supported_device(self, device: InputDevice) -> bool:
        if self.device_name_contains:
            needle = self.device_name_contains.lower()
            if needle not in device.name.lower():
                return False

        capabilities = device.capabilities()
        if ecodes.EV_ABS not in capabilities:
            return False

        abs_codes = self.flatten_codes(capabilities[ecodes.EV_ABS])
        return ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes

    @staticmethod
    def flatten_codes(codes: Iterable[object]) -> set:
        flattened = set()
        for code in codes:
            if isinstance(code, tuple):
                flattened.add(code[0])
            else:
                flattened.add(code)
        return flattened

    def init_axis(self, device: InputDevice) -> None:
        capabilities = device.capabilities(absinfo=True)
        if ecodes.EV_ABS not in capabilities:
            return

        self.axis_center.clear()
        self.axis_min.clear()
        self.axis_max.clear()

        for code, info in capabilities[ecodes.EV_ABS]:
            self.axis_min[code] = float(info.min)
            self.axis_max[code] = float(info.max)
            self.axis_center[code] = (float(info.max) + float(info.min)) / 2.0

    def normalize(self, code: int, value: int) -> float:
        minv = self.axis_min.get(code, 0.0)
        maxv = self.axis_max.get(code, 1.0)
        if maxv <= minv:
            return 0.0

        if code in [ecodes.ABS_Z, ecodes.ABS_RZ]:
            normalized = (float(value) - minv) / (maxv - minv)
            return 0.0 if normalized < 0.05 else normalized

        center = self.axis_center.get(code, (maxv + minv) / 2.0)
        if value >= center:
            denominator = maxv - center
            normalized = (
                0.0 if denominator == 0.0 else (float(value) - center) / denominator
            )
        else:
            denominator = center - minv
            normalized = (
                0.0 if denominator == 0.0 else (float(value) - center) / denominator
            )

        return 0.0 if abs(normalized) < self.deadzone else normalized

    def print_mapping(self) -> None:
        self.get_logger().info("===== AXES =====")
        for index, name in self.axis_name.items():
            self.get_logger().info(f"axes[{index}] = {name}")

        self.get_logger().info("===== BUTTONS =====")
        for index, name in self.button_name.items():
            self.get_logger().info(f"buttons[{index}] = {name}")

    def read_events(self) -> None:
        device = self.device
        if device is None:
            return

        try:
            for event in device.read_loop():
                if event.type == ecodes.EV_ABS and event.code in self.axis_map:
                    index = self.axis_map[event.code]
                    value = self.normalize(event.code, event.value)
                    with self.lock:
                        self.msg.axes[index] = value
                elif event.type == ecodes.EV_KEY and event.code in self.button_map:
                    index = self.button_map[event.code]
                    with self.lock:
                        self.msg.buttons[index] = int(event.value)
        except OSError as exc:
            self.get_logger().warn(f"Joystick disconnected: {exc}")
        finally:
            try:
                device.close()
            except OSError:
                pass
            if self.device is device:
                self.device = None

    def publish_joy(self) -> None:
        with self.lock:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.axes = list(self.msg.axes)
            msg.buttons = list(self.msg.buttons)

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JljWholebodyJoystickNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node.device is not None:
            node.device.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
