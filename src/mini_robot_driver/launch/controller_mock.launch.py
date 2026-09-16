import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_package_share = get_package_share_directory("mini_robot_urdf")
    driver_package_share = get_package_share_directory("mini_robot_driver")
    xacro_path = os.path.join(urdf_package_share, "xacro", "mini_robot.xacro")
    controllers_path = os.path.join(driver_package_share, "config", "controllers.yaml")

    robot_description = xacro.process_file(xacro_path).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[controllers_path],
                remappings=[("robot_description", "/robot_description")],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
