from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'min_height': 0.45,
                'max_height': 1.0,
                'range_min': 0.6,
                'use_sim_time': True,
                'qos_overrides./scan.publisher.reliability': 'best_effort'
            }]
        )
    ])