from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lanekeeping',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            emulate_tty=True,
        )
    ])
