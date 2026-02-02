from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package="serial_test",
        executable="test_node",
        name="test_node",
        output="screen"
    )
    ld.add_action(node)

    node_joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )
    ld.add_action(node_joy)

    # usb_port = LaunchConfiguration('port')
    # ld.add_action(
    #     DeclareLaunchArgument(
    #         'port',
    #         default_value='/dev/ttyUSB0',
    #         description='USB serial port for FSM/bridge',
    #     )
    # )
    # fsm_node = Node(
    #     package='serial_test',
    #     executable='serial_comm_fsm_node',
    #     name='serial_comm_fsm_node',
    #     output='screen',
    #     parameters=[
    #         {'port': usb_port},
    #     ],
    # )
    # ld.add_action(fsm_node)

    # Path to the URDF file
    pkg_path = os.path.join(get_package_share_directory('amr_demo'))
    pkg_path2 = os.path.join(get_package_share_directory('serial_test'))
    urdf_file = os.path.join(pkg_path, 'description', 'amr_demo_center.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    ld.add_action(node_robot_state_publisher)

    # Add rviz node
    # node_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(pkg_path2, 'rviz', 'amr_rviz.rviz')]
    # )
    # ld.add_action(node_rviz)

    # Include the sllidar_s2_2.launch file from another package
    # sllidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_s2_2_launch.py')
    #     ])
    # )
    # ld.add_action(sllidar_launch)

    return ld
