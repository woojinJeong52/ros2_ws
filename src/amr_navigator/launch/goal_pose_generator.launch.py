from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_file = LaunchConfiguration('output_file')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    name_prefix = LaunchConfiguration('name_prefix')
    lookup_timeout_sec = LaunchConfiguration('lookup_timeout_sec')
    float_precision = LaunchConfiguration('float_precision')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_file',
            default_value='',
            description='Waypoint YAML path. Empty uses src/amr_navigator/params/waypoints.yaml when launched from the workspace.',
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('name_prefix', default_value='work_station'),
        DeclareLaunchArgument('lookup_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('float_precision', default_value='6'),
        Node(
            package='amr_navigator',
            executable='goal_pose_generator',
            name='goal_pose_generator',
            output='screen',
            parameters=[
                {'output_file': output_file},
                {'map_frame': map_frame},
                {'base_frame': base_frame},
                {'name_prefix': name_prefix},
                {'lookup_timeout_sec': ParameterValue(lookup_timeout_sec, value_type=float)},
                {'float_precision': ParameterValue(float_precision, value_type=int)},
            ],
        ),
    ])
