from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='lifecycle',
        description='Task scenario: production, recycling, lifecycle',
    )
    stage_arg = DeclareLaunchArgument(
        'stage',
        default_value='beginner',
        description='Task stage: entry, beginner, or advanced',
    )
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/eai/task',
        description='Topic used by task publisher and listener',
    )
    side_a_topic_name_arg = DeclareLaunchArgument(
        'side_a_topic_name',
        default_value='/eai/task/side_a',
        description='Topic used by side_a task publisher',
    )
    side_b_topic_name_arg = DeclareLaunchArgument(
        'side_b_topic_name',
        default_value='/eai/task/side_b',
        description='Topic used by side_b task publisher',
    )
    publish_period_sec_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Periodic publish interval in seconds',
    )
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='false',
        description='Publish once and exit when true',
    )

    task_publisher_node = Node(
        package='eai_task_server',
        executable='task_publisher',
        name='task_publisher',
        output='screen',
        parameters=[
            {
                'scenario': LaunchConfiguration('scenario'),
                'stage': LaunchConfiguration('stage'),
                'topic_name': LaunchConfiguration('topic_name'),
                'side_a_topic_name': LaunchConfiguration('side_a_topic_name'),
                'side_b_topic_name': LaunchConfiguration('side_b_topic_name'),
                'publish_period_sec': LaunchConfiguration('publish_period_sec'),
                'publish_once': LaunchConfiguration('publish_once'),
            }
        ],
    )

    task_listener_node = Node(
        package='eai_task_server',
        executable='task_listener',
        name='task_listener',
        output='screen',
        parameters=[
            {
                'topic_name': LaunchConfiguration('topic_name'),
            }
        ],
    )

    return LaunchDescription(
        [
            scenario_arg,
            stage_arg,
            topic_name_arg,
            side_a_topic_name_arg,
            side_b_topic_name_arg,
            publish_period_sec_arg,
            publish_once_arg,
            task_publisher_node,
            task_listener_node,
        ]
    )
