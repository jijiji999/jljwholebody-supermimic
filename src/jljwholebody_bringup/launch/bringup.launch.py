from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    motor_config_path = LaunchConfiguration("motor_config_path")
    parallel_mechanisms_path = LaunchConfiguration("parallel_mechanisms_path")
    enable_on_activate = LaunchConfiguration("enable_on_activate")
    disable_on_deactivate = LaunchConfiguration("disable_on_deactivate")
    command_timeout_ms = LaunchConfiguration("command_timeout_ms")
    feedback_poll_timeout_ms = LaunchConfiguration("feedback_poll_timeout_ms")
    lifecycle_timeout_ms = LaunchConfiguration("lifecycle_timeout_ms")
    startup_feedback_warmup_s = LaunchConfiguration("startup_feedback_warmup_s")
    enable_retries = LaunchConfiguration("enable_retries")
    enable_retry_interval_ms = LaunchConfiguration("enable_retry_interval_ms")
    controller_manager_path = LaunchConfiguration("controller_manager_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")
    use_imu = LaunchConfiguration("use_imu")
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

    description_pkg = FindPackageShare("jljwholebody_description")
    bringup_pkg = FindPackageShare("jljwholebody_bringup")
    hardware_pkg = FindPackageShare("jljwholebody_hardware")

    xacro_file = PathJoinSubstitution(
        [description_pkg, "urdf", "jljwholebody.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [bringup_pkg, "config", "controllers.yaml"]
    )
    default_parallel_mechanisms_path = PathJoinSubstitution(
        [hardware_pkg, "config", "parallel_mechanisms.yaml"]
    )
    default_motor_config_path = PathJoinSubstitution(
        [hardware_pkg, "config", "dm_motor_whole_body.yaml"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_mock_hardware:=",
                use_mock_hardware,
                " motor_config_path:=",
                motor_config_path,
                " parallel_mechanisms_path:=",
                parallel_mechanisms_path,
                " enable_on_activate:=",
                enable_on_activate,
                " disable_on_deactivate:=",
                disable_on_deactivate,
                " command_timeout_ms:=",
                command_timeout_ms,
                " feedback_poll_timeout_ms:=",
                feedback_poll_timeout_ms,
                " lifecycle_timeout_ms:=",
                lifecycle_timeout_ms,
                " startup_feedback_warmup_s:=",
                startup_feedback_warmup_s,
                " enable_retries:=",
                enable_retries,
                " enable_retry_interval_ms:=",
                enable_retry_interval_ms,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_path,
        ],
        output="screen",
    )

    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--controller-manager",
            controller_manager_path,
        ],
        output="screen",
    )

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
                "publish_rate": ParameterValue(
                    joystick_publish_rate,
                    value_type=float,
                ),
                "reconnect_period": ParameterValue(
                    joystick_reconnect_period,
                    value_type=float,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument(
                "motor_config_path", default_value=default_motor_config_path
            ),
            DeclareLaunchArgument(
                "parallel_mechanisms_path",
                default_value=default_parallel_mechanisms_path,
            ),
            DeclareLaunchArgument("enable_on_activate", default_value="false"),
            DeclareLaunchArgument("disable_on_deactivate", default_value="true"),
            DeclareLaunchArgument("command_timeout_ms", default_value="0"),
            DeclareLaunchArgument("feedback_poll_timeout_ms", default_value="0"),
            DeclareLaunchArgument("lifecycle_timeout_ms", default_value="500"),
            DeclareLaunchArgument("startup_feedback_warmup_s", default_value="0.5"),
            DeclareLaunchArgument("enable_retries", default_value="3"),
            DeclareLaunchArgument("enable_retry_interval_ms", default_value="200"),
            DeclareLaunchArgument("controller_manager_path", default_value="/controller_manager"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_joystick", default_value="true"),
            DeclareLaunchArgument("use_imu", default_value="false"),
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument(
                "joystick_device_path",
                default_value=EnvironmentVariable(
                    "JLJWHOLEBODY_JOY_DEVICE",
                    default_value="",
                ),
            ),
            DeclareLaunchArgument(
                "joystick_device_name",
                default_value=EnvironmentVariable(
                    "JLJWHOLEBODY_JOY_NAME",
                    default_value="",
                ),
            ),
            DeclareLaunchArgument("joystick_deadzone", default_value="0.10"),
            DeclareLaunchArgument("joystick_publish_rate", default_value="50.0"),
            DeclareLaunchArgument("joystick_reconnect_period", default_value="2.0"),
            DeclareLaunchArgument("joystick_frame_id", default_value="flydigi"),
            DeclareLaunchArgument(
                "imu_port",
                default_value=EnvironmentVariable(
                    "JLJWHOLEBODY_IMU_PORT",
                    default_value="/dev/dm_imu",
                ),
            ),
            DeclareLaunchArgument(
                "imu_baud",
                default_value=EnvironmentVariable(
                    "JLJWHOLEBODY_IMU_BAUD",
                    default_value="921600",
                ),
            ),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_publish_pose", default_value="false"),
            robot_state_publisher,
            ros2_control_node,
            joystick_node,
            imu_node,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=3.0, actions=[whole_body_controller_spawner]),
        ]
    )
