from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_robot_tf',
            executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster',
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
