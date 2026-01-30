import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchService, LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node as LaunchNode
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import asyncio

class LineManager(Node):
    def __init__(self):
        super().__init__('line_manager')
        self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        self.status = ''
        
        self.launch_service = LaunchService()
        self.launch_description = LaunchDescription()
        self.running_nodes = {}

        # LaunchService를 별도의 비동기 태스크로 실행
        asyncio.ensure_future(self.launch_service.run_async())
        self.get_logger().info('LineManager node initialized')

    def control_from_center_callback(self, msg):
        self.status = msg.data
        self.get_logger().info(f'Received message: {self.status}')
        self.read_status(self.status)

    def read_status(self, msg):
        valid_msgs = ['ch2_start', 'lu1_start', 'lqr_start', 'lu2_start', 'ld1_start', 'ch1_start']
        alias_msgs = ['ch2', 'lu1', 'lqr', 'lu2', 'ld1', 'ch1']
        
        if msg in valid_msgs:
            self.get_logger().info(f'Valid message received: {msg}')
            for m in valid_msgs:
                if m == msg:
                    alias = alias_msgs[valid_msgs.index(m)]
                    self.start_node(alias)
                else:
                    self.terminate_node(alias)
        else:
            self.get_logger().warn('Wrong status message input!!')

    def start_node(self, alias):
        if alias not in self.running_nodes:
            package_share_directory = get_package_share_directory('line_manager')
            executable_name = f'{alias}_node_executable'  # 실제 실행 파일 이름으로 변경 필요
            node = LaunchNode(
                package='line_manager',
                executable=executable_name,
                name=alias,
                output='screen'
            )
            self.launch_description.add_action(node)
            self.running_nodes[alias] = node
            self.launch_service.include_launch_description(self.launch_description)
            self.get_logger().info(f'{alias} started')
        else:
            self.get_logger().info(f'{alias} is already running')

    def terminate_node(self, alias):
        if alias in self.running_nodes:
            node = self.running_nodes.pop(alias)
            self.launch_description.add_action(Shutdown())
            self.get_logger().info(f'{alias} terminated')
            self.launch_service.include_launch_description(self.launch_description)
            asyncio.ensure_future(self.launch_service.run_async())

async def main_async():
    rclpy.init()
    node = LineManager()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_async())
    loop.close()

if __name__ == '__main__':
    main()
