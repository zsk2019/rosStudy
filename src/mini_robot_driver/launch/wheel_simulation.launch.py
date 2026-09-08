import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_package_share = get_package_share_directory("mini_robot_urdf")
    xacro_path = os.path.join(urdf_package_share, "xacro", "mini_robot.xacro")
    rviz_config_path = os.path.join(urdf_package_share, "rviz", "mini_robot.rviz")

    robot_description = xacro.process_file(xacro_path).toxml()

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    left_wheel_joint_name = LaunchConfiguration("left_wheel_joint_name")
    right_wheel_joint_name = LaunchConfiguration("right_wheel_joint_name")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    use_rviz = LaunchConfiguration("use_rviz")

    common_wheel_parameters = {
        "wheel_radius": wheel_radius,
        "wheel_separation": wheel_separation,
        "left_wheel_joint_name": left_wheel_joint_name,
        "right_wheel_joint_name": right_wheel_joint_name,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.05",
                description="Wheel radius in meters.",
            ),
            DeclareLaunchArgument(
                "wheel_separation",
                default_value="0.44",
                description="Distance between left and right wheels in meters.",
            ),
            DeclareLaunchArgument(
                "left_wheel_joint_name",
                default_value="left_wheel_joint",
                description="Left wheel joint name.",
            ),
            DeclareLaunchArgument(
                "right_wheel_joint_name",
                default_value="right_wheel_joint",
                description="Right wheel joint name.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame id.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Base frame id.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="mini_robot_driver",
                executable="diff_drive_simulator_node",
                name="diff_drive_simulator",
                parameters=[common_wheel_parameters],
                output="screen",
            ),
            Node(
                package="mini_robot_driver",
                executable="wheel_encoder_simulator_node",
                name="wheel_encoder_simulator",
                parameters=[
                    {
                        "left_wheel_joint_name": left_wheel_joint_name,
                        "right_wheel_joint_name": right_wheel_joint_name,
                    }
                ],
                output="screen",
            ),
            Node(
                package="mini_robot_driver",
                executable="wheel_odometry",
                name="wheel_odometry",
                parameters=[
                    common_wheel_parameters,
                    {
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                    },
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
