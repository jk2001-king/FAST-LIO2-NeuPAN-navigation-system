import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    base_model_dir = get_package_share_directory('base_model')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = 'src/nav2/2dmap/fastlio_map_2d_2.yaml'
    params_file = 'src/nav2/config/nav2_params_amcl.yaml'
    rviz_config_path = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_model_dir, 'launch', 'scout_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'target_frame': 'base_link',
                'transform_tolerance': 0.2,
                'min_height': 0.05,
                'max_height': 1.20,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.2,
                'range_min': 0.05,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }
        ],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/scan'),
        ],
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription(
        [
            gazebo_launch,
            TimerAction(period=2.0, actions=[pointcloud_to_scan]),
            TimerAction(period=4.0, actions=[localization_launch]),
            TimerAction(period=6.0, actions=[navigation_launch, rviz_node]),
        ]
    )
