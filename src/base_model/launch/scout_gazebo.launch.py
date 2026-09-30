import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_base_model = get_package_share_directory('base_model')
    spawn_x = '-40.0'
    spawn_y = '0.0'
    spawn_z = '0.35'
    spawn_yaw = '0.0'
    
    try:
        pkg_scout_desc = get_package_share_directory('scout_description')
        scout_render_path = os.path.join(pkg_scout_desc, '..') 
    except:
        scout_render_path = os.path.join(pkg_base_model, '..')

    gazebo_models_path = os.path.join(os.path.expanduser('~'), '.gazebo', 'models')
    
    final_model_path = f"{gazebo_models_path}:{scout_render_path}"

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=final_model_path
    )
    
    default_world_path = 'src/base_model/worlds/end_feature_wide_mid.world'
    world_path = LaunchConfiguration('world_path')
    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=default_world_path,
        description='Gazebo world file path'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    xacro_file = os.path.join(pkg_base_model, 'urdf', 'scout_v2.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scout_bot', 
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
        output='screen'
    )

    return LaunchDescription([
        declare_world_path,
        set_gazebo_model_path,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
