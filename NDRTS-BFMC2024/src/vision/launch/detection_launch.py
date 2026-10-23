from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='custom_detect_node',
            name='traffic_sign_node',
            output='screen',
            emulate_tty=True,  # Useful for seeing print() logs
        )
    ])
