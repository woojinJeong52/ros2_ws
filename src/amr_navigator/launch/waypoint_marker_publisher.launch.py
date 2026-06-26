from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    waypoints_file = LaunchConfiguration('waypoints_file')
    marker_topic = LaunchConfiguration('marker_topic')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    marker_scale = LaunchConfiguration('marker_scale')
    text_scale = LaunchConfiguration('text_scale')

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description='Waypoint YAML path. Empty uses src/robocup_navigator/params/stations_robocup.yaml when launched from the workspace.',
        ),
        DeclareLaunchArgument('marker_topic', default_value='/waypoint_markers'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
        DeclareLaunchArgument('marker_scale', default_value='0.22'),
        DeclareLaunchArgument('text_scale', default_value='0.22'),
        Node(
            package='amr_navigator',
            executable='waypoint_marker_publisher',
            name='waypoint_marker_publisher',
            output='screen',
            parameters=[
                {'waypoints_file': waypoints_file},
                {'marker_topic': marker_topic},
                {'publish_period_sec': ParameterValue(publish_period_sec, value_type=float)},
                {'marker_scale': ParameterValue(marker_scale, value_type=float)},
                {'text_scale': ParameterValue(text_scale, value_type=float)},
            ],
        ),
    ])
