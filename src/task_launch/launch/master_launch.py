from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rob_pkg_launch_dir = os.path.join(get_package_share_directory('carla_ros_bridge'))
    wp_pkg_launch_dir = os.path.join(get_package_share_directory('carla_waypoint_publisher'))
    master_launch_dir =  os.path.join(get_package_share_directory('task_launch'), 'launch')
    
    # Get the HOME directory
    home_dir = os.environ["HOME"]

    # Construct full path to your JSON file
    vehicle_def_file_path = os.path.join(home_dir, "CARLA_PROJECT/carla_ros2_ws/src/Self-Driving-Car-CARLA-ROS2/car_definition_file.json")


    ego_vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(rob_pkg_launch_dir,'carla_ros_bridge_with_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': vehicle_def_file_path,
        }.items()
    )

    way_point_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(wp_pkg_launch_dir,'carla_waypoint_publisher.launch.py')
        )
    )

    control_navig_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(master_launch_dir,'multi_node_launch.py')
        )
    )

    return LaunchDescription([
        ego_vehicle_launch,
        way_point_launch,
        control_navig_launch
    ])



