from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_path = LaunchConfiguration("config_path")
    use_joystick = LaunchConfiguration("use_joystick")
    use_imu = LaunchConfiguration("use_imu")
    validate_config_only = LaunchConfiguration("validate_config_only")
    joy_topic = LaunchConfiguration("joy_topic")
    joystick_device_path = LaunchConfiguration("joystick_device_path")
    joystick_device_name = LaunchConfiguration("joystick_device_name")
    joystick_deadzone = LaunchConfiguration("joystick_deadzone")
    joystick_publish_rate = LaunchConfiguration("joystick_publish_rate")
    joystick_reconnect_period = LaunchConfiguration("joystick_reconnect_period")
    joystick_frame_id = LaunchConfiguration("joystick_frame_id")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_pose = LaunchConfiguration("imu_publish_pose")

    joystick_node = Node(
        package="jljwholebody_joystick",
        executable="joystick_node",
        name="joystick",
        condition=IfCondition(use_joystick),
        parameters=[
            {
                "joy_topic": joy_topic,
                "device_path": joystick_device_path,
                "device_name_contains": joystick_device_name,
                "deadzone": ParameterValue(joystick_deadzone, value_type=float),
                "publish_rate": ParameterValue(joystick_publish_rate, value_type=float),
                "reconnect_period": ParameterValue(
                    joystick_reconnect_period, value_type=float
                ),
                "frame_id": joystick_frame_id,
            }
        ],
        output="screen",
    )

    imu_node = Node(
        package="dm_imu",
        executable="dm_imu_node",
        name="dm_imu_node",
        condition=IfCondition(use_imu),
        parameters=[
            {
                "port": imu_port,
                "baud": ParameterValue(imu_baud, value_type=int),
                "frame_id": imu_frame_id,
                "publish_pose": ParameterValue(imu_publish_pose, value_type=bool),
            }
        ],
        output="screen",
    )

    deploy_node = Node(
        package="jljwholebody_deploy",
        executable="jljwholebody_deploy_node",
        name="jljwholebody_deploy",
        parameters=[
            {
                "config_path": config_path,
                "validate_config_only": ParameterValue(
                    validate_config_only, value_type=bool
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("use_joystick", default_value="true"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("validate_config_only", default_value="false"),
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("joystick_device_path", default_value=""),
            DeclareLaunchArgument("joystick_device_name", default_value=""),
            DeclareLaunchArgument("joystick_deadzone", default_value="0.10"),
            DeclareLaunchArgument("joystick_publish_rate", default_value="50.0"),
            DeclareLaunchArgument("joystick_reconnect_period", default_value="2.0"),
            DeclareLaunchArgument("joystick_frame_id", default_value="flydigi"),
            DeclareLaunchArgument("imu_port", default_value="/dev/dm_imu"),
            DeclareLaunchArgument("imu_baud", default_value="921600"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_publish_pose", default_value="false"),
            joystick_node,
            imu_node,
            deploy_node,
        ]
    )
