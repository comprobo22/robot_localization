from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    map_file = DeclareLaunchArgument('map_yaml')

    lifecycle_nodes = ['map_server']
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="true")
    autostart = True

    start_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    return LaunchDescription([
        map_file,
        use_sim_time,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{"yaml_filename": LaunchConfiguration('map_yaml')}],
            output='screen'
        ),
        start_lifecycle_manager
    ])
