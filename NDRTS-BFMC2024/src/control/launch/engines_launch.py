from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Ensures logs are flushed immediately (similar to PYTHONUNBUFFERED=1)
        Node(
            package='control',
            executable='engine_node',
            name='engines_node',
            output='screen'
        ),

        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen'
        ),

        Node(
            package='control',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 115200
            }]
        ),

        Node(
            package='control',
            executable='stop_lanekeeping_manager_node',
            name='stop_lanekeeping_manager_node',
            output='screen'
        ),

        # Node(
        #     package='control',
        #     executable='monitor_scan_node',
        #     name='monitor_scan',
        #     output='screen'
        # ),
    ])
