"""Standalone NeuPAN Node Launch File for FAST-LIO2 Integration."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.logging import get_logger
from launch_ros.actions import Node

logger = get_logger('neupan_IICC_launch')

def generate_launch_description() -> LaunchDescription:
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(pkg_share, 'config', 'robots', 'scout')
    robot_config = os.path.join(robot_config_dir, 'robot.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'neupan_sim.rviz')

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
            {'scan_topic': '/livox/lidar'},
            # Fallback mode runs in odom. Do not feed Nav2's map-frame /plan
            # directly into NeuPAN, or its initial path will be built in the
            # wrong frame and drag the robot toward obstacles.
            {'plan_input_topic': '/neupan_plan_input_odom'},
            {'refresh_initial_path': True},
            # 기본은 TF를 그대로 신뢰 (후진/축 뒤집힘이 보일 때만 true로 테스트)
            {'yaw_flip_180': False},
            {'invert_cmd_linear_x': False},
            {'invert_cmd_angular_z': False},
            {'ignore_goal_orientation': True},
            # NeuPAN 출력 안정화 (제자리 빙빙 방지)
            {'max_abs_angular_z': 0.8},
            {'turn_slowdown_gain': 1.5},
            {'max_linear_x': 0.6},
            {'min_linear_x': 0.05},
        ],
        remappings=[
            # NeuPAN 내부 파라미터 기본값이 "/neupan_cmd_vel" 이지만,
            # 노드 구현/사용 환경에 따라 절대/상대 이름이 달라질 수 있어 둘 다 커버합니다.
            ('/neupan_cmd_vel', '/cmd_vel_neupan'),
            ('neupan_cmd_vel', '/cmd_vel_neupan'),
            ('/goal_pose', '/neupan/final_goal'),
            # ⭐ 오도메트리 토픽도 현재 시스템이 뱉는 이름으로 맞춰야 합니다.
            # 만약 odom_switcher가 /neupan/current_odom을 뱉고 있다면 유지, 
            # 아니면 실제 오도메트리 토픽(/odom 등)으로 맞춰주세요.
            ('/Odometry', '/neupan/current_odom') 
        ]
    )

    if not os.path.exists(rviz_config):
        rviz_args = []
    else:
        rviz_args = ['-d', rviz_config]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=rviz_args,
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        neupan_node,
        rviz_node,
    ])
