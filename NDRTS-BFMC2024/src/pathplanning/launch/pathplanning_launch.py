# pathplanning/launch/pathplanning_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    graphml_name = 'traseu.graphml'  # ← change this when needed

    graphml_path = os.path.join(
        os.getenv('HOME'),
        'workspaces/NDRTS-BFMC2024/src/pathplanning/config',
        graphml_name
    )

    return LaunchDescription([
        Node(
            package='pathplanning',
            executable='path_publisher_node',
            name='path_publisher_node',
            parameters=[
                {'graphml_file': graphml_path},
                # optional, in case node also needs name
                {'graphml_name': graphml_name}
            ],
            output='screen',
        ),
        Node(
            package='pathplanning',
            executable='gps_position_node',
            name='gps_position_node',
            output='screen',
        ),
        Node(
            package='pathplanning',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[
                {'graphml_file': graphml_path},
                # optional, in case node also needs name
                {'graphml_name': graphml_name}
            ],
            output='screen',
        ),
    ])
