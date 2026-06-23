from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('service_name', default_value='/get_target_pose'),
        DeclareLaunchArgument('camera_mode', default_value='mid_50'),
        DeclareLaunchArgument('brick_search_mode', default_value='fine'),
        DeclareLaunchArgument('local_id', default_value='0'),
        DeclareLaunchArgument('visualize_capture', default_value='false'),
        DeclareLaunchArgument('visualize_search', default_value='false'),
        DeclareLaunchArgument('debug_summary', default_value='true'),
        DeclareLaunchArgument('yolo_device', default_value='auto'),
        Node(
            package='vision_revise_pkg',
            executable='vision_node',
            name='vision_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'service_name': LaunchConfiguration('service_name'),
                'camera_mode': LaunchConfiguration('camera_mode'),
                'brick_search_mode': LaunchConfiguration('brick_search_mode'),
                'local_id': LaunchConfiguration('local_id'),
                'visualize_capture': LaunchConfiguration('visualize_capture'),
                'visualize_search': LaunchConfiguration('visualize_search'),
                'debug_summary': LaunchConfiguration('debug_summary'),
                'yolo_device': LaunchConfiguration('yolo_device'),
            }],
        ),
    ])
