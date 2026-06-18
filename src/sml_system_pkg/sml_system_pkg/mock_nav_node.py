"""
mock_nav_node.py
navigate_to_station Action 서버 mock.
goal 수신 → MOVING 피드백 → delay 후 success=True 반환.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sml_msgs.action import NavTask

NAV_DELAY = 1.0  # 이동 시뮬레이션 시간 (초)


class MockNavNode(Node):

    def __init__(self):
        super().__init__('mock_nav_node')
        self.cbg = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            NavTask,
            'navigate_to_station',
            execute_callback=self._execute_cb,
            callback_group=self.cbg,
        )
        self.get_logger().info('[MOCK NAV] navigate_to_station 서버 시작')

    def _execute_cb(self, goal_handle):
        station_id = goal_handle.request.station_id
        self.get_logger().info(
            f'[MOCK NAV] goal 수신: station_id={station_id}')

        # 피드백: MOVING
        fb = NavTask.Feedback()
        fb.status = 'MOVING'
        goal_handle.publish_feedback(fb)

        time.sleep(NAV_DELAY)

        # 피드백: ARRIVED
        fb.status = 'ARRIVED'
        goal_handle.publish_feedback(fb)

        goal_handle.succeed()

        result = NavTask.Result()
        result.success = True
        result.fail_reason = ''
        self.get_logger().info(
            f'[MOCK NAV] 완료: station_id={station_id}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockNavNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
