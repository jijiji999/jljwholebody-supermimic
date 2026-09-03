from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_topic = LaunchConfiguration("joy_topic")
    device_path = LaunchConfiguration("device_path")
    device_name_contains = LaunchConfiguration("device_name_contains")
    deadzone = LaunchConfiguration("deadzone")
    publish_rate = LaunchConfiguration("publish_rate")
    reconnect_period = LaunchConfiguration("reconnect_period")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("device_path", default_value=""),
            DeclareLaunchArgument("device_name_contains", default_value=""),
            DeclareLaunchArgument("deadzone", default_value="0.10"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("reconnect_period", default_value="2.0"),
            DeclareLaunchArgument("frame_id", default_value="flydigi"),
            Node(
                package="jljwholebody_joystick",
                executable="joystick_node",
                name="joystick",
                output="screen",
                parameters=[
                    {
                        "joy_topic": joy_topic,
                        "device_path": device_path,
                        "device_name_contains": device_name_contains,
                        "deadzone": ParameterValue(deadzone, value_type=float),
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "reconnect_period": ParameterValue(
                            reconnect_period,
                            value_type=float,
                        ),
                        "frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
