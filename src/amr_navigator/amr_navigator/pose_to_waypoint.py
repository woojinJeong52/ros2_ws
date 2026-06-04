import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseToWaypoint(Node):
    def __init__(self):
        super().__init__('pose_to_waypoint')

        self.declare_parameter('input_topic', '/goal_pose')
        self.declare_parameter('input_type', 'pose_stamped')
        self.declare_parameter('waypoint_name', 'new_waypoint')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('once', True)

        self.done = False
        self._once = bool(self.get_parameter('once').value)
        self._waypoint_name = self.get_parameter('waypoint_name').value
        input_topic = self.get_parameter('input_topic').value
        input_type = self.get_parameter('input_type').value

        if input_type == 'pose_stamped':
            self.create_subscription(PoseStamped, input_topic, self._pose_stamped_callback, 10)
        elif input_type == 'pose_with_covariance_stamped':
            self.create_subscription(
                PoseWithCovarianceStamped,
                input_topic,
                self._pose_with_covariance_callback,
                10,
            )
        else:
            raise ValueError(
                'input_type must be "pose_stamped" or "pose_with_covariance_stamped"'
            )

        self.get_logger().info(
            f'Waiting for {input_type} on {input_topic}. '
            'Use RViz2 "2D Goal Pose" for /goal_pose.'
        )

    def _pose_stamped_callback(self, msg: PoseStamped):
        self._print_pose(msg.pose, msg.header.frame_id)

    def _pose_with_covariance_callback(self, msg: PoseWithCovarianceStamped):
        self._print_pose(msg.pose.pose, msg.header.frame_id)

    def _print_pose(self, pose: Pose, msg_frame_id: str):
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        yaw_deg = math.degrees(yaw)
        frame_id = self.get_parameter('frame_id').value or msg_frame_id or 'map'

        print()
        print('--- pose ---')
        print(f'frame_id: {frame_id}')
        print(f'x: {pose.position.x:.6f}')
        print(f'y: {pose.position.y:.6f}')
        print(f'yaw_rad: {yaw:.6f}')
        print(f'yaw_deg: {yaw_deg:.2f}')
        print()
        print('--- waypoint yaml ---')
        print(f'{self._waypoint_name}:')
        print(
            '  position: '
            f'{{x: {pose.position.x:.6f}, y: {pose.position.y:.6f}, z: {pose.position.z:.6f}}}'
        )
        print(
            '  orientation: '
            f'{{x: {pose.orientation.x:.8f}, y: {pose.orientation.y:.8f}, '
            f'z: {pose.orientation.z:.8f}, w: {pose.orientation.w:.8f}}}'
        )
        print()

        if self._once:
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = PoseToWaypoint()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
