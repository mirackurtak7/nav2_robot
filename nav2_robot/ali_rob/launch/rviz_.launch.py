#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000')  # for S2
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')
    gui = LaunchConfiguration('gui', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    urdf_file_name = 'dolly.urdf'
    urdf = os.path.join(
        get_package_share_directory('ali_rob'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen',
        respawn=True,
        respawn_delay=1.0
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time
        }],
        respawn=True,
        respawn_delay=1.0
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'map_topic': '/map'
        }],
        respawn=True,
        respawn_delay=1.0
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying USB port to connect lidar'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying USB port baudrate to connect lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value=gui,
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Flag to enable use_sim_time'
        ),

        LogInfo(
            condition=IfCondition(gui),
            msg=['Launching with GUI enabled']
        ),
        LogInfo(
            condition=UnlessCondition(gui),
            msg=['Launching without GUI']
        ),
        LogInfo(
            msg=['Launching rplidar_node with channel_type: ', channel_type,
                 ', serial_port: ', serial_port, ', serial_baudrate: ', serial_baudrate]
        ),

        lidar_node,
        # RF2O Laser Odometry Node
        rf2o_node,
        
        # SLAM Toolbox Node
        slam_toolbox_node,

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            respawn=True,
            respawn_delay=1.0
        ),

        # Robot State Publisher Node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            respawn=True,
            respawn_delay=1.0
        ),
    
        # Joint State Publisher Node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
            respawn=True,
            respawn_delay=1.0
        ),
    
        # Joint State Publisher GUI Node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            respawn=True,
            respawn_delay=1.0
        )
    ])

