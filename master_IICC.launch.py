import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    enable_hybrid = LaunchConfiguration('enable_hybrid')
    pcd_map_path = 'pcd/map_lite2.pcd'
    yaml_map_file = 'src/nav2/2dmap/fastlio_map_2d_2.yaml'
    rviz_config_path = 'IICC_rviz_nav2.rviz'
    nav2_params_file = 'src/nav2/config/nav2_params_full.yaml'

    declare_enable_hybrid_cmd = DeclareLaunchArgument(
        'enable_hybrid', default_value='true',
        description='Launch slam_toggle/NeuPAN hybrid navigation stack'
    )

    # --- 1. 가제보 & 센서 필터 ---
    base_model_dir = get_package_share_directory('base_model')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_model_dir, 'launch', 'scout_gazebo.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    pc_fixer = ExecuteProcess(cmd=['python3', 'src/base_model/base_model/pointcloud_fixer.py', '--ros-args', '-p', 'use_sim_time:=true'], output='screen')

    # --- 2. 로컬라이제이션 (정근님 원본 - 알아서 맵 띄우고 다리 놓음) ---
    loc_dir = get_package_share_directory('fast_lio_localization')
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_dir, 'launch', 'velodyne_localization.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'config_file': 'mid360.yaml',
            'map_path': pcd_map_path,
            'rviz': 'false',
        }.items()
    )

    map_server = Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[{'yaml_filename': yaml_map_file}, {'use_sim_time': True}])
    lifecycle_manager_map = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': ['map_server']}])
    
    # --- 3. Nav2 및 RViz ---
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_path], parameters=[{'use_sim_time': True}])
    nav2_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_params_file, 'autostart': 'true'}.items()
    )

    # --- 4. 주행 신경망 (Mux & Bridge) 및 NeuPAN ---
    cmd_vel_mux = Node(
        package='slam_toggle',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        parameters=[
            {'use_sim_time': True},
            {'fallback_to_neupan_if_nav2_stale': True},
            {'override_timeout_sec': 0.25},
            {'source_timeout_sec': 0.25},
            {'publish_rate_hz': 50.0},
        ],
        output='screen',
        condition=IfCondition(enable_hybrid),
    )
    relay_node = Node(package='topic_tools', executable='relay', name='cmd_vel_relay', arguments=['/cmd_vel_neupan', '/cmd_vel'], parameters=[{'use_sim_time': True}], output='screen', condition=IfCondition(enable_hybrid))
    toggle_manager = Node(
        package='slam_toggle',
        executable='toggle_manager',
        parameters=[
            {'use_sim_time': True},
            {'arrive_distance_threshold': 0.9},
            {'arrive_confirm_sec': 1.0},
            {'publish_recovery_initialpose': False},
            {'recovery_realign_threshold': 0.45},
            {'recovery_goal_resend_delay_sec': 0.8},
        ],
        condition=IfCondition(enable_hybrid),
    )
    odom_switcher = Node(package='slam_toggle', executable='odom_switcher', parameters=[{'use_sim_time': True}], condition=IfCondition(enable_hybrid))
    auto_toggle = Node(
        package='slam_toggle',
        executable='auto_toggle',
        parameters=[
            {'use_sim_time': True},
            {'enable_auto': True},
            {'w_res': 0.4},
            {'w_vel': 0.4},
            {'w_z': 0.2},
            {'s_norm': 0.15},
            {'v_norm': 0.5},
            {'z_norm': 0.25},
            {'degrade_threshold': 1.0},
            {'recover_threshold': 0.7},
            {'localization_fail_penalty': 0.6},
            {'localization_false_streak_threshold': 3},
            {'localization_true_streak_recover_threshold': 3},
            {'c_degrad_ema_alpha': 0.25},
            {'z_baseline_alpha': 0.05},
            {'enable_fitness_gate': True},
            {'fitness_degrade_threshold': 0.35},
            {'fitness_recover_threshold': 0.39},
            {'degrade_confirm_sec': 1.0},
            {'recover_confirm_sec': 1.5},
            {'arming_delay_sec': 5.0},
            {'require_fitness_for_loc_bad': True},
            {'require_localization_ok_for_recovery': False},
            {'require_alignment_for_recovery': True},
            {'recovery_align_threshold': 0.30},
        ],
        condition=IfCondition(enable_hybrid),
    )

    neupan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('neupan_ros2'), 'launch', 'neupan_IICC.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'use_rviz': 'false'}.items(),
        condition=IfCondition(enable_hybrid),
    )
    qos_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('neupan_ros2'), 'launch', 'qos_bridge.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(enable_hybrid),
    )

    # --- 🎬 최종 실행 시퀀스 ---
    return LaunchDescription([
        declare_enable_hybrid_cmd,
        gazebo_launch,
        pc_fixer,
        
        TimerAction(period=3.0, actions=[map_server, lifecycle_manager_map]),
        TimerAction(period=7.0, actions=[localization_launch, rviz_node]),
        
        TimerAction(period=10.0, actions=[toggle_manager, odom_switcher, auto_toggle, cmd_vel_mux]),
        
        TimerAction(period=13.0, actions=[GroupAction([SetRemap(src='/cmd_vel', dst='/cmd_vel_nav2'), nav2_core])]),
        TimerAction(period=16.0, actions=[qos_bridge_launch, relay_node, GroupAction([SetRemap(src='/cmd_vel', dst='/cmd_vel_neupan'), neupan_launch])])
    ])
