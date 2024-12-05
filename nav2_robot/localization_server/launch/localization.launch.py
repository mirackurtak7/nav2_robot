import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'atolye.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    return LaunchDescription([
        
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time':False},
                        {'yaml_filename':map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time':False},
                        {'autostart':True},
                        {'node_names':['map_server','amcl']}]
        )

    ])
