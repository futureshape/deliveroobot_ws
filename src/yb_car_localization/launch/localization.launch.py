"""
Complete launch file for YB_Car localization with robot description and visualization.
Launches robot_state_publisher, EKF sensor fusion, and optionally SLAM or just RViz2.

Modes:
  - Localization only (enable_slam:=false): EKF + odometry fusion
  - SLAM mapping (enable_slam:=true): EKF + SLAM Toolbox for map building
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directory
    pkg_share = get_package_share_directory('yb_car_localization')
    
    # Paths to config files
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'yb_car.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'yb_car_view.rviz')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')
    slam_rviz_config = os.path.join(pkg_share, 'config', 'slam_view.rviz')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    enable_slam = LaunchConfiguration('enable_slam')
    
    return LaunchDescription([
        
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable SLAM mapping mode (true) or localization only (false)'
        ),
        
        # Robot State Publisher - publishes TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
            }]
        ),
        
        # Joint State Publisher - publishes wheel joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # EKF Node for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),
        
        # SLAM Toolbox - Only when enable_slam:=true
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(enable_slam)
        ),
        
        # RViz2 for localization visualization (when SLAM disabled)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_rviz)
        ),
    ])
