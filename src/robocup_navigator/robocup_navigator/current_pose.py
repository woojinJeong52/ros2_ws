import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CurrentPosePrinter(Node):
    def __init__(self):
        super().__init__('robocup_current_pose')

        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('waypoint_name', 'new_waypoint')
        self.declare_parameter('timeout_sec', 3.0)
        self.declare_parameter('tf_sample_count', 5)
        self.declare_parameter('tf_sample_period_sec', 0.1)
        self.declare_parameter('max_tf_position_jump_m', 0.10)
        self.declare_parameter('max_tf_yaw_jump_rad', 0.35)

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
        sample_count = max(1, int(self.get_parameter('tf_sample_count').value))
        sample_period_sec = max(
            0.0,
            float(self.get_parameter('tf_sample_period_sec').value),
        )

        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        samples = []

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
                samples.append(transform)

                if len(samples) >= sample_count:
                    if not self._validate_tf_samples(samples):
                        return False
                    self._print_transform(samples[-1])
                    return True

                rclpy.spin_once(self, timeout_sec=sample_period_sec)
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def _validate_tf_samples(self, samples) -> bool:
        if len(samples) <= 1:
            return True

        max_position_jump_m = float(
            self.get_parameter('max_tf_position_jump_m').value
        )
        max_yaw_jump_rad = float(
            self.get_parameter('max_tf_yaw_jump_rad').value
        )
        first = samples[0].transform
        first_yaw = quaternion_to_yaw(
            first.rotation.x,
            first.rotation.y,
            first.rotation.z,
            first.rotation.w,
        )

        largest_position_jump = 0.0
        largest_yaw_jump = 0.0

        for sample in samples[1:]:
            current = sample.transform
            position_jump = math.hypot(
                current.translation.x - first.translation.x,
                current.translation.y - first.translation.y,
            )
            yaw = quaternion_to_yaw(
                current.rotation.x,
                current.rotation.y,
                current.rotation.z,
                current.rotation.w,
            )
            yaw_jump = abs(normalize_angle(yaw - first_yaw))

            largest_position_jump = max(largest_position_jump, position_jump)
            largest_yaw_jump = max(largest_yaw_jump, yaw_jump)

        if (largest_position_jump > max_position_jump_m
                or largest_yaw_jump > max_yaw_jump_rad):
            self.get_logger().error(
                'Unstable TF samples. '
                f'position_jump={largest_position_jump:.3f} m, '
                f'yaw_jump={largest_yaw_jump:.3f} rad. '
                'Check localization convergence or duplicate TF publishers.'
            )
            return False

        return True

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
        qx = q.x
        qy = q.y
        qz = q.z
        qw = q.w

        if qw < 0.0:
            qx = -qx
            qy = -qy
            qz = -qz
            qw = -qw

        yaw = quaternion_to_yaw(qx, qy, qz, qw)

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
            f'{{x: {qx:.16f}, y: {qy:.16f}, '
            f'z: {qz:.16f}, w: {qw:.16f}}}'
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
