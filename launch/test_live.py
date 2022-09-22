from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions.launch_configuration_equals import LaunchConfigurationEquals
from launch.conditions.launch_configuration_not_equals import LaunchConfigurationNotEquals
import os

def generate_launch_description():
    map_file = DeclareLaunchArgument('map_yaml')
    use_built_in = DeclareLaunchArgument('use_built_in', default_value='true')

    lifecycle_nodes = ['map_server']
    lifecycle_nodes_built_in = ['map_server', 'amcl']

    # TODO: no support currently for not using sim time
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_built_in = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            condition=LaunchConfigurationEquals('use_built_in', 'true'),
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_built_in}])

    start_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            condition=LaunchConfigurationNotEquals('use_built_in', 'true'),
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])


    return LaunchDescription([
        map_file,
        use_built_in,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{"yaml_filename": LaunchConfiguration('map_yaml')}],
            output='screen'
        ),
        Node(package='nav2_amcl',
             executable='amcl',
             name='amcl',
             parameters=[{'use_sim_time': use_sim_time}],
             condition=LaunchConfigurationEquals(use_built_in, 'true'),
             output='screen'),
        Node(package='robot_localization',
             executable='pf.py',
             name='my_pf',
             parameters=[{'use_sim_time': use_sim_time}],
             output='screen',
             condition=LaunchConfigurationNotEquals(use_built_in, 'true')),
        start_lifecycle_manager_built_in,
        start_lifecycle_manager
    ])
