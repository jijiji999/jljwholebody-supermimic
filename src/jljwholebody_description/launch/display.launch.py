from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    description_pkg = FindPackageShare("jljwholebody_description")
    xacro_file = PathJoinSubstitution(
        [description_pkg, "urdf", "jljwholebody.urdf.xacro"]
    )
    rviz_file = PathJoinSubstitution([description_pkg, "rviz", "view_robot.rviz"])

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_mock_hardware:=",
                use_mock_hardware,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_gui),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_file],
                output="screen",
            ),
        ]
    )
