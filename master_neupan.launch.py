import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_base_model = get_package_share_directory('base_model')
    pkg_neupan = get_package_share_directory('neupan_ros2')

    spawn_x = '-27.0'
    spawn_y = '0.0'
    spawn_z = '0.35'
    spawn_yaw = '0.0'

    try:
        pkg_scout_desc = get_package_share_directory('scout_description')
        scout_render_path = os.path.join(pkg_scout_desc, '..')
    except Exception:
        scout_render_path = os.path.join(pkg_base_model, '..')

    gazebo_models_path = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    final_model_path = f"{gazebo_models_path}:{scout_render_path}"

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=final_model_path,
    )

    world_path = 'src/base_model/worlds/feature_corridor.world'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    xacro_file = os.path.join(pkg_base_model, 'urdf', 'scout_v2.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'publish_odom_tf': 'true'},
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'scout_bot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    robot_config_dir = os.path.join(pkg_neupan, 'config', 'robots', 'scout')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')

    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_config,
            {'robot_config_dir': robot_config_dir},
            {'use_sim_time': True},
            {'map_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'scan_topic': '/livox/lidar'},
            {'goal_topic': '/goal_pose'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'yaw_flip_180': False},
            {'invert_cmd_linear_x': False},
            {'invert_cmd_angular_z': False},
            {'ignore_goal_orientation': True},
            {'max_abs_angular_z': 0.6},
            {'turn_slowdown_gain': 1.5},
            {'max_linear_x': 0.5},
            {'min_linear_x': 0.05},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_neupan, 'rviz', 'neupan.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
        TimerAction(period=3.0, actions=[neupan_node]),
        TimerAction(period=5.0, actions=[rviz_node]),
    ])
