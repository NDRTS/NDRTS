# graph_nav2/launch/bringup_launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_graph = get_package_share_directory('graph_nav2')

    # ---------- default files ----------
    default_map = os.path.join(pkg_graph, 'maps',   'blank_map.yaml')
    default_params = os.path.join(pkg_graph, 'config', 'nav2_params.yaml')
    default_pubkey = os.path.join(
        pkg_graph, 'useful', 'publickey_server_test.pem')

    # ---------- launch-file arguments ----------
    map_arg = LaunchConfiguration('map')
    params_arg = LaunchConfiguration('params_file')
    pubkey_arg = LaunchConfiguration('pubkey_file')

    return LaunchDescription([

        # 0) declare args *first*
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to a map YAML file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Absolute path to a Nav2 parameters YAML file'),
        DeclareLaunchArgument(
            'pubkey_file',
            default_value=default_pubkey,
            description='Server RSA public-key PEM used by traffic_comm'),

        # 1) helper nodes ------------------------------------------------
        Node(
            package='graph_nav2', executable='graph_loader',
            name='graph_loader', output='screen',
            parameters=[{
                'graphml_file': os.path.join(pkg_graph, 'maps', 'map.graphml')
            }]),

        Node(package='graph_nav2', executable='gps_localizer',
             name='gps_localizer', output='screen'),

        Node(package='graph_nav2', executable='motor_controller',
             name='motor_controller', output='screen'),

        Node(package='graph_nav2', executable='waypoint_navigator',
             name='waypoint_navigator', output='screen'),

        # 2) static TF ---------------------------------------------------
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']),

        # 3) traffic-communication node ----------------------------------
        Node(
            package='graph_nav2', executable='traffic_comm',
            name='traffic_comm', output='screen',
            parameters=[{
                'device_id': 1,
                'frequency': 0.4,         # Hz
                'pem_path': pubkey_arg    # <-- matches traffic_communication_node.py
            }]),

        # 4) Nav2 bring-up ----------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'bringup_launch.py')),
            launch_arguments={
                'map':         map_arg,
                'use_sim_time': 'false',
                'params_file': params_arg,
            }.items()),
    ])
