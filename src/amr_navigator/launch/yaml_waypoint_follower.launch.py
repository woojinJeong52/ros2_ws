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
    arrive_topic = LaunchConfiguration('arrive_topic')
    done_topic = LaunchConfiguration('done_topic')
    arrive_prefix = LaunchConfiguration('arrive_prefix')
    done_prefix = LaunchConfiguration('done_prefix')
    wait_for_done = LaunchConfiguration('wait_for_done')
    require_done_match = LaunchConfiguration('require_done_match')
    continue_on_miss = LaunchConfiguration('continue_on_miss')

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
        DeclareLaunchArgument(
            'arrive_topic',
            default_value='serial_tx',
            description='Topic to publish arrival flags',
        ),
        DeclareLaunchArgument(
            'done_topic',
            default_value='serial_rx',
            description='Topic to subscribe for done flags',
        ),
        DeclareLaunchArgument(
            'arrive_prefix',
            default_value='ARRIVED',
            description='Prefix for arrival flag (e.g. ARRIVED:point_name)',
        ),
        DeclareLaunchArgument(
            'done_prefix',
            default_value='DONE',
            description='Prefix for done flag (e.g. DONE:point_name)',
        ),
        DeclareLaunchArgument(
            'wait_for_done',
            default_value='true',
            description='Wait for done flag before moving to next waypoint',
        ),
        DeclareLaunchArgument(
            'require_done_match',
            default_value='true',
            description='Require done flag to include current waypoint name',
        ),
        DeclareLaunchArgument(
            'continue_on_miss',
            default_value='false',
            description='Continue sequence even if a waypoint is missed',
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
                {'arrive_topic': arrive_topic},
                {'done_topic': done_topic},
                {'arrive_prefix': arrive_prefix},
                {'done_prefix': done_prefix},
                {'wait_for_done': wait_for_done},
                {'require_done_match': require_done_match},
                {'continue_on_miss': continue_on_miss},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[],
        ),
    ])
