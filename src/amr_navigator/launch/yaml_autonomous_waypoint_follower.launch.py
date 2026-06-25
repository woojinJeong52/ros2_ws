# ros2 launch amr_navigator yaml_autonomous_waypoint_follower.launch.py waypoints_file:=/home/moonshot/ros2_ws/src/amr_navigator/params/waypoints.yaml

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('amr_navigator'))
    default_yaml = str(pkg_share / 'params' / 'waypoints.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')
    sequence = LaunchConfiguration('sequence')
    frame_id = LaunchConfiguration('frame_id')
    auto_start = LaunchConfiguration('auto_start')
    repeat = LaunchConfiguration('repeat')
    repeat_delay_sec = LaunchConfiguration('repeat_delay_sec')
    continue_on_miss = LaunchConfiguration('continue_on_miss')
    follow_waypoints_action_name = LaunchConfiguration('follow_waypoints_action_name')
    follow_waypoints_server_timeout_sec = LaunchConfiguration(
        'follow_waypoints_server_timeout_sec'
    )
    status_topic = LaunchConfiguration('status_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_yaml,
            description='Path to waypoints YAML file.',
        ),
        DeclareLaunchArgument(
            'sequence',
            default_value='',
            description='Optional waypoint sequence override, e.g. [a,b].',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Default frame ID when YAML does not define frame_id.',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Start waypoint navigation immediately.',
        ),
        DeclareLaunchArgument(
            'repeat',
            default_value='false',
            description='Repeat all YAML waypoints after completion.',
        ),
        DeclareLaunchArgument(
            'repeat_delay_sec',
            default_value='0.0',
            description='Delay before repeating all waypoints.',
        ),
        DeclareLaunchArgument(
            'continue_on_miss',
            default_value='false',
            description='Treat missed waypoints as non-fatal after Nav2 returns.',
        ),
        DeclareLaunchArgument(
            'follow_waypoints_action_name',
            default_value='follow_waypoints',
            description='Nav2 FollowWaypoints action name.',
        ),
        DeclareLaunchArgument(
            'follow_waypoints_server_timeout_sec',
            default_value='10.0',
            description='Timeout while waiting for FollowWaypoints server.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='',
            description='Optional std_msgs/String status topic.',
        ),
        Node(
            package='amr_navigator',
            executable='yaml_autonomous_waypoint_node',
            name='yaml_autonomous_waypoint_follower',
            output='screen',
            parameters=[
                {'waypoints_file': waypoints_file},
                {'sequence': ParameterValue(sequence, value_type=str)},
                {'frame_id': frame_id},
                {'auto_start': auto_start},
                {'repeat': repeat},
                {'repeat_delay_sec': repeat_delay_sec},
                {'continue_on_miss': continue_on_miss},
                {'follow_waypoints_action_name': follow_waypoints_action_name},
                {'follow_waypoints_server_timeout_sec': follow_waypoints_server_timeout_sec},
                {'status_topic': ParameterValue(status_topic, value_type=str)},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
