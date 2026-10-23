from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def pkg_launch(pkg: str, file: str) -> str:
    """Absolute path to <pkg>/launch/<file> inside its share directory."""
    return os.path.join(
        get_package_share_directory(pkg), 'launch', file
    )


def generate_launch_description():
    return LaunchDescription([
        # Vision Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('vision', 'camera_launch.py'))
        ),

        # Vision Custom Detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('vision', 'detection_launch.py'))
        ),
        # Lane Keeping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('lanekeeping', 'lanekeeping_launch.py'))
        ),

        # RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('rplidar_ros', 'rplidar_a2m8_launch.py'))
        ),

        # Engine Control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('control', 'engines_launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                pkg_launch('graph_nav2', 'bringup_launch.py'))
        ),

        # Optional: Add this if you re-enable pathplanning
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         pkg_launch('pathplanning', 'pathplanning_launch.py'))
        # ),

        # Optional: Add this if you re-enable dashboard
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('dashboard'), 'launch', 'dashboard_launch.py'))
        # ),
    ])
