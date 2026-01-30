from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_charge_out_node',
            name='ch2',
            output='screen'
        ),
        
    
        Node(
            package='line_follower',
            executable='line_lift_up_node',
            name='lu1',
            output='screen'
        ),

        Node(
            package='check_leg',
            executable='check_node',
            name='lqr',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_lift_up_out_node',
            name='lu2',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_lift_down_node',
            name='ld1',
            output='screen'
        ),

        Node(
            package='line_follower',
            executable='line_charge_node',
            name='ch1',
            output='screen'
        ),





    ])
