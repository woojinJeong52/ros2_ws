#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sml_messages.msg import Order, Task


ORDER_TYPE_NAMES = {
    Order.OT_PRODUCE: 'PRODUCE',
    Order.OT_RECYCLE: 'RECYCLE',
}


class TaskListenerNode(Node):
    def __init__(self) -> None:
        super().__init__('task_listener')
        self.declare_parameter('topic_name', '/eai/task')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self._subscription = self.create_subscription(Task, topic_name, self._task_callback, 10)

        self.get_logger().info(f'Listening for tasks on {topic_name}')

    def _task_callback(self, msg: Task) -> None:
        self.get_logger().info(
            f'Received task: {len(msg.order_list)} orders, {len(msg.arena_layout)} stations'
        )

        for order in msg.order_list:
            order_type_name = ORDER_TYPE_NAMES.get(order.order_type, f'UNKNOWN({order.order_type})')
            self.get_logger().info(
                f'  Order name={order.name}, type={order_type_name}, product_id={order.product_id}'
            )

        for station in msg.arena_layout:
            self.get_logger().info(
                f'  Station name={station.name}, id={station.station_id}, '
                f'type={station.station_type}, materials={list(station.material_ids)}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskListenerNode()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
