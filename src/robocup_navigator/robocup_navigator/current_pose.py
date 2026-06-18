import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CurrentPosePrinter(Node):
    def __init__(self):
        super().__init__('robocup_current_pose')

        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('waypoint_name', 'new_waypoint')
        self.declare_parameter('timeout_sec', 3.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_odom = None

    def print_current_pose(self) -> bool:
        pose_source = str(self.get_parameter('pose_source').value).lower()
        if pose_source == 'tf':
            return self._print_tf_pose()
        if pose_source == 'odom':
            return self._print_odom_pose()

        self.get_logger().error(
            f'Invalid pose_source="{pose_source}". Use "tf" or "odom".'
        )
        return False

    def _print_tf_pose(self) -> bool:
        target_frame = self.get_parameter('target_frame').value
        source_frame = self.get_parameter('source_frame').value
        timeout_sec = float(self.get_parameter('timeout_sec').value)

        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec

        while rclpy.ok():
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec > deadline:
                self.get_logger().error(
                    f'Failed to get transform {target_frame} -> '
                    f'{source_frame} within {timeout_sec:.1f} sec.'
                )
                return False

            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
                self._print_transform(transform)
                return True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def _print_odom_pose(self) -> bool:
        odom_topic = self.get_parameter('odom_topic').value
        timeout_sec = float(self.get_parameter('timeout_sec').value)
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec

        subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )

        while rclpy.ok():
            if self._latest_odom is not None:
                self._print_odom(self._latest_odom)
                self.destroy_subscription(subscription)
                return True

            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec > deadline:
                self.get_logger().error(
                    f'Failed to receive odometry from "{odom_topic}" '
                    f'within {timeout_sec:.1f} sec.'
                )
                self.destroy_subscription(subscription)
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(subscription)
        return False

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _print_transform(self, transform):
        waypoint_name = self.get_parameter('waypoint_name').value
        target_frame = transform.header.frame_id
        t = transform.transform.translation
        q = transform.transform.rotation
        self._print_waypoint_yaml(waypoint_name, target_frame, t, q)

    def _print_odom(self, odom):
        waypoint_name = self.get_parameter('waypoint_name').value
        frame_id = odom.header.frame_id
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        self._print_waypoint_yaml(waypoint_name, frame_id, position, orientation)
        print(f'# child_frame_id: {odom.child_frame_id}')

    def _print_waypoint_yaml(self, waypoint_name, frame_id, position,
                             orientation):
        t = position
        q = orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        print(f'frame_id: {frame_id}')
        print()
        print('waypoints:')
        print(f'  {waypoint_name}:')
        print(
            '    position: '
            f'{{x: {t.x:.16f}, y: {t.y:.16f}, z: {t.z:.16f}}}'
        )
        print(
            '    orientation: '
            f'{{x: {q.x:.16f}, y: {q.y:.16f}, '
            f'z: {q.z:.16f}, w: {q.w:.16f}}}'
        )
        print()
        print(f'# yaw_rad: {yaw:.16f}')
        print(f'# yaw_deg: {math.degrees(yaw):.6f}')


def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosePrinter()

    try:
        ok = node.print_current_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
