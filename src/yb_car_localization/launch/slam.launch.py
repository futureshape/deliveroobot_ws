#!/usr/bin/env python3
"""
Launch SLAM Toolbox for mapping with the YB_Car robot.

This launch file starts:
  - EKF sensor fusion (fuses IMU + odometry)
  - SLAM Toolbox in online async mapping mode  
  - Robot state publisher (URDF/TF)
  - Optional: RViz for visualization

The EKF provides the odom_frame->base_footprint transform using fused sensor data.
SLAM Toolbox then adds the map->odom_frame transform to complete the chain.
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('yb_car_localization')
    
    # Paths to config files
    urdf_file = os.path.join(pkg_share, 'urdf', 'yb_car.urdf.xacro')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'slam_view.rviz')
    
    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether to launch RViz for visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot State Publisher - publishes TF transforms from URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0
        }]
    )
    
    # EKF Node - Fuses IMU + Odometry and publishes odom_frame->base_footprint transform
    # This provides smooth, accurate odometry for SLAM to use
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # SLAM Toolbox - Online Async Mapping (uses /odometry/filtered from EKF)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz - Optional visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        launch_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        ekf_node,
        slam_toolbox_node,
        rviz_node
    ])
