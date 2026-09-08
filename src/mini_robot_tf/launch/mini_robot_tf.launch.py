from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_odom_tf_broadcaster = LaunchConfiguration('use_odom_tf_broadcaster')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_odom_tf_broadcaster',
            default_value='false',
            description='Whether to start the standalone odom -> base_link TF demo',
        ),
        Node(
            package='mini_robot_tf',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
            condition=IfCondition(use_odom_tf_broadcaster),
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'linear_velocity': 0.2,
                'angular_velocity': 0.4,
                'publish_period': 0.05,
            }],
        ),
        Node(
            package='mini_robot_tf',
            executable='sensor_tf_broadcaster',
            name='sensor_tf_broadcaster',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'laser_frame': 'laser_link',
                'camera_frame': 'camera_link',
            }],
        ),
    ])
