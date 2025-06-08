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
            parameters=[{'disable_control': True}],
            namespace='SELF_DRIVE_stack',
            output='screen'
        ),
        Node(
            package='front_collison_alert',
            executable='monitor_col_zone',
            name='Automatic_emergency_breaking',
            namespace='SELF_DRIVE_stack',
            output='screen'
        ),
        # Add more nodes similarly
    ])
