# dashboard/launch/dashboard_launch.py
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = Path(get_package_share_directory('dashboard'))

    node_server = share / 'server' / 'ros_server.js'
    vite_root = share / 'webapp' / 'dashboard-ui'          # uses installed assets

    return LaunchDescription([
        ExecuteProcess(                                      # Node-ROS bridge
            cmd=['node', str(node_server)],
            name='ros_websocket_bridge',
            output='screen'),

        ExecuteProcess(                                      # Vite dev server
            cmd=['npm', 'run', 'dev', '--', '--host'],
            cwd=str(vite_root),
            name='vite_dev_server',
            output='screen'),
    ])
