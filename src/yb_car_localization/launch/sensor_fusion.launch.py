"""
Launch file for YB_Car sensor fusion using robot_localization EKF node.
Fuses IMU and odometry data for robust pose estimation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # EKF Node for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                LaunchConfiguration('ekf_config', default=[
                    LaunchConfiguration('yb_car_localization_dir', 
                                      default='install/yb_car_localization/share/yb_car_localization'),
                    '/config/ekf.yaml'
                ])
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),
    ])
