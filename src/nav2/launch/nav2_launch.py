import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_nav_dir = get_package_share_directory('nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = 'src/nav2/2dmap/fastlio_map_2d_2.yaml'
    params_file = os.path.join(my_nav_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file},
                        {'use_sim_time': True}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true'
            }.items(),
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='map_to_odom_bridge',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # ),
        # # nav2_launch.py 맨 밑에 추가 (임시)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='fake_bridge',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        # ),
    ])
