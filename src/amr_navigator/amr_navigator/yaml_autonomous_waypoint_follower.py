from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header, String


class YamlAutonomousWaypointFollower(Node):
    def __init__(self):
        super().__init__('yaml_autonomous_waypoint_follower')

        default_yaml = str(
            Path(get_package_share_directory('amr_navigator'))
            / 'params'
            / 'waypoints.yaml'
        )

        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('sequence', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('repeat', False)
        self.declare_parameter('repeat_delay_sec', 0.0)
        self.declare_parameter('continue_on_miss', False)
        self.declare_parameter('follow_waypoints_action_name', 'follow_waypoints')
        self.declare_parameter('follow_waypoints_server_timeout_sec', 10.0)
        self.declare_parameter('status_topic', '')

        action_name = self.get_parameter('follow_waypoints_action_name').value
        self._action_client = ActionClient(self, FollowWaypoints, action_name)

        status_topic = str(self.get_parameter('status_topic').value)
        self._status_pub = (
            self.create_publisher(String, status_topic, 10)
            if status_topic
            else None
        )

        self._goal_handle = None
        self._is_running = False
        self._repeat_timer = None

        self._waypoint_names, self._waypoints = self._load_waypoints()

        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoint(s): {self._waypoint_names}'
        )

        if self.get_parameter('auto_start').value:
            self.start()
        else:
            self.get_logger().info('auto_start is false; waiting for manual start.')

    def _load_waypoints(self) -> Tuple[List[str], List[PoseStamped]]:
        path = Path(str(self.get_parameter('waypoints_file').value)).expanduser()
        frame_id = str(self.get_parameter('frame_id').value)

        if not path.exists():
            self.get_logger().error(f'Waypoints file not found: {path}')
            return [], []

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            return [], []

        if not isinstance(data, dict):
            self.get_logger().error('Waypoints YAML root must be a mapping.')
            return [], []

        if data.get('frame_id'):
            frame_id = str(data['frame_id'])

        waypoints_map = data.get('waypoints', {})
        if not isinstance(waypoints_map, dict):
            self.get_logger().error('Waypoints YAML "waypoints" must be a mapping.')
            return [], []

        sequence = self._resolve_sequence(data, waypoints_map)
        poses = []
        valid_names = []

        for name in sequence:
            entry = waypoints_map.get(name)
            if entry is None:
                self.get_logger().warn(f'Waypoint "{name}" not found in YAML; skipping.')
                continue

            pose = self._entry_to_pose(name, entry, frame_id)
            if pose is None:
                self.get_logger().warn(f'Waypoint "{name}" invalid; skipping.')
                continue

            poses.append(pose)
            valid_names.append(name)

        return valid_names, poses

    def _resolve_sequence(self, data: Dict, waypoints_map: Dict) -> List[str]:
        seq_param = self.get_parameter('sequence').value

        if seq_param:
            return self._parse_sequence(seq_param)

        yaml_sequence = data.get('sequence')
        if yaml_sequence:
            if isinstance(yaml_sequence, list):
                return [str(item) for item in yaml_sequence]

            self.get_logger().warn(
                'YAML "sequence" is not a list; using waypoints mapping order.'
            )

        return [str(name) for name in waypoints_map.keys()]

    def _parse_sequence(self, value) -> List[str]:
        if isinstance(value, list):
            return [str(item) for item in value]

        text = str(value).strip()
        if not text:
            return []

        try:
            parsed = yaml.safe_load(text)
        except Exception:
            parsed = None

        if isinstance(parsed, list):
            return [str(item) for item in parsed]

        return [item.strip() for item in text.strip('[]').split(',') if item.strip()]

    def _entry_to_pose(
        self,
        name: str,
        entry: Dict,
        frame_id: str,
    ) -> Optional[PoseStamped]:
        try:
            if not isinstance(entry, dict):
                raise ValueError('waypoint entry must be a mapping')

            if 'pose' in entry:
                pose_list = entry['pose']
                if len(pose_list) != 7:
                    raise ValueError('pose list must have 7 elements')
                x, y, z, qx, qy, qz, qw = pose_list
            else:
                position = entry.get('position', {})
                orientation = entry.get('orientation', {})

                x = position.get('x')
                y = position.get('y')
                z = position.get('z', 0.0)

                qx = orientation.get('x', 0.0)
                qy = orientation.get('y', 0.0)
                qz = orientation.get('z', 0.0)
                qw = orientation.get('w', 1.0)

            if x is None or y is None:
                raise ValueError('position.x and position.y are required')

            return PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=frame_id,
                ),
                pose=Pose(
                    position=Point(
                        x=float(x),
                        y=float(y),
                        z=float(z),
                    ),
                    orientation=Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw),
                    ),
                ),
            )
        except Exception as exc:
            self.get_logger().error(f'Invalid waypoint "{name}": {exc}')
            return None

    def start(self):
        if self._is_running:
            self.get_logger().warn('Waypoint navigation is already running.')
            return

        if not self._waypoints:
            self.get_logger().warn('No valid waypoints configured.')
            return

        timeout_sec = float(
            self.get_parameter('follow_waypoints_server_timeout_sec').value
        )
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                'FollowWaypoints action server unavailable; navigation not started.'
            )
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self._refresh_pose_stamps(self._waypoints)

        self._is_running = True
        self._publish_status('driving_start')
        self.get_logger().info(
            f'Sending {len(goal_msg.poses)} waypoint(s) to FollowWaypoints.'
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _refresh_pose_stamps(self, poses: List[PoseStamped]) -> List[PoseStamped]:
        stamp = self.get_clock().now().to_msg()
        refreshed = []

        for pose in poses:
            updated = PoseStamped()
            updated.header.frame_id = pose.header.frame_id
            updated.header.stamp = stamp
            updated.pose = pose.pose
            refreshed.append(updated)

        return refreshed

    def _goal_response_callback(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send waypoint goal: {exc}')
            self._finish_running('driving_failed')
            return

        if not self._goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected.')
            self._finish_running('driving_rejected')
            return

        self.get_logger().info('Waypoint goal accepted.')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            result = future.result().result
            missed = list(result.missed_waypoints)
        except Exception as exc:
            self.get_logger().error(f'Failed to receive waypoint result: {exc}')
            self._finish_running('driving_failed')
            return

        if missed:
            missed_names = [
                self._waypoint_names[index]
                for index in missed
                if 0 <= index < len(self._waypoint_names)
            ]
            self.get_logger().warn(
                f'Missed waypoint index(es): {missed}, name(s): {missed_names}'
            )

            if not bool(self.get_parameter('continue_on_miss').value):
                self._finish_running('driving_missed')
                return

        self.get_logger().info('All configured waypoints processed.')
        self._finish_running('driving_done')

        if bool(self.get_parameter('repeat').value):
            self._schedule_repeat()

    def _finish_running(self, status: str):
        self._goal_handle = None
        self._is_running = False
        self._publish_status(status)

    def _publish_status(self, status: str):
        if self._status_pub is None:
            return

        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _schedule_repeat(self):
        delay = float(self.get_parameter('repeat_delay_sec').value)

        def restart():
            if self._repeat_timer is not None:
                self._repeat_timer.cancel()
                self._repeat_timer = None
            self.start()

        if delay <= 0.0:
            restart()
        else:
            self._repeat_timer = self.create_timer(delay, restart)


def main(args=None):
    rclpy.init(args=args)
    node = YamlAutonomousWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
