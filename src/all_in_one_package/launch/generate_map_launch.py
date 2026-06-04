from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # First Launch: serial_test
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('serial_test')),
                '/serial_test.launch.py'])
        ),

        # Second Launch: sllidar_ros2 after 5 seconds
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('sllidar_ros2'), 'launch'),
                        '/sllidar_s2_2_launch.py'])
                ),
            ]
        ),

        # Third Launch: ros2_laser_scan_merger after 10 seconds
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros2_laser_scan_merger'), 'launch'),
                        '/merge_2_scan.launch.py'])
                ),
            ]
        ),

        # Fourth Launch: Cartographer SLAM after 6 seconds
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('amr_cartographer'),
                            'launch',
                            'amr_cartographer.launch.py'
                        )
                    ),
                ),
            ]
        ),

    ])
