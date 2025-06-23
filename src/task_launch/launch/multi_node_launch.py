from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carla_ad_agent',
            executable='local_planner',
            name='local_planner',
            namespace='SELF_DRIVE_stack',
            output='screen'
        ),
        Node(
            package='navigation',
            executable='navigation_hmi',
            name='navigation_hmi',
            parameters=[{'enable_spd_cmd': False}],
            namespace='SELF_DRIVE_stack',
            output='screen'
        ),
        Node(
            package='collision_avoidance_system',
            executable='collision_monitor_control',
            name='collision_monitor_control',
            namespace='SELF_DRIVE_stack',
            output='screen'
        ),
        # Add more nodes similarly
    ])
