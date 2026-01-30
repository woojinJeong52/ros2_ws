from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')
    default_yaml = os.path.join(pkg_share, 'params', 'waypoints.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')
    sequence = LaunchConfiguration('sequence')
    frame_id = LaunchConfiguration('frame_id')
    auto_start = LaunchConfiguration('auto_start')
    repeat = LaunchConfiguration('repeat')
    repeat_delay_sec = LaunchConfiguration('repeat_delay_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_yaml,
            description='Path to waypoints YAML file',
        ),
        DeclareLaunchArgument(
            'sequence',
            default_value='',
            description='Override waypoint sequence (e.g. [d1,d2,d3])',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for waypoints',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start sequence immediately',
        ),
        DeclareLaunchArgument(
            'repeat',
            default_value='false',
            description='Repeat sequence after completion',
        ),
        DeclareLaunchArgument(
            'repeat_delay_sec',
            default_value='0.0',
            description='Delay before repeating sequence (seconds)',
        ),
        Node(
            package='amr_navigator',
            executable='yaml_waypoint_node',
            name='yaml_waypoint_follower',
            output='screen',
            parameters=[
                {'waypoints_file': waypoints_file},
                {'frame_id': frame_id},
                {'auto_start': auto_start},
                {'repeat': repeat},
                {'repeat_delay_sec': repeat_delay_sec},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[],
        ),
    ])
