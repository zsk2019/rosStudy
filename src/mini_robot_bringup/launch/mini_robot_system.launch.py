from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_monitor = LaunchConfiguration("use_monitor")
    robot_driver_config = PathJoinSubstitution(
        [FindPackageShare("mini_robot_bringup"), "config", "robot_driver.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_id",
                default_value="mini_robot_01",
                description="Robot identifier used by robot_controller_node",
            ),
            DeclareLaunchArgument(
                "use_monitor",
                default_value="true",
                description="Whether to start robot_monitor_node",
            ),
            Node(
                package="mini_robot_driver",
                executable="robot_monitor_node",
                name="robot_monitor_node",
                condition=IfCondition(use_monitor),
                output="screen",
            ),
            Node(
                package="mini_robot_driver",
                executable="robot_controller_node",
                name="robot_controller_node",
                parameters=[robot_driver_config, {"robot_id": robot_id}],
                output="screen",
            ),
        ]
    )
