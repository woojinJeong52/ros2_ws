import os
from typing import Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
import yaml


class YamlWaypointFollower(Node):
    def __init__(self):
        super().__init__('yaml_waypoint_follower')
        default_yaml = os.path.join(
            get_package_share_directory('amr_navigator'),
            'params',
            'waypoints.yaml',
        )
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('sequence', [])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('repeat', False)
        self.declare_parameter('repeat_delay_sec', 0.0)

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._goal_handle = None
        self._goal_accept = False
        self._repeat_timer = None

        self._waypoints_map, self._sequence, self._frame_id = self._load_waypoints()

        if self.get_parameter('auto_start').value:
            self._start_sequence()
        else:
            self.get_logger().info('auto_start is false; waiting for manual start.')

    def _load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        frame_id = self.get_parameter('frame_id').value
        if not os.path.exists(path):
            self.get_logger().error(f'Waypoints file not found: {path}')
            return {}, [], frame_id

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            return {}, [], frame_id

        if isinstance(data, dict) and data.get('frame_id'):
            frame_id = data.get('frame_id')

        waypoints_map = data.get('waypoints', {}) if isinstance(data, dict) else {}

        seq_param = self.get_parameter('sequence').value
        if seq_param:
            if isinstance(seq_param, str):
                try:
                    parsed = yaml.safe_load(seq_param)
                    if isinstance(parsed, list):
                        sequence = parsed
                    else:
                        cleaned = seq_param.strip().strip('[]')
                        sequence = [item.strip() for item in cleaned.split(',') if item.strip()]
                except Exception:
                    cleaned = seq_param.strip().strip('[]')
                    sequence = [item.strip() for item in cleaned.split(',') if item.strip()]
            else:
                sequence = list(seq_param)
        else:
            sequence = data.get('sequence', list(waypoints_map.keys())) if isinstance(data, dict) else []

        return waypoints_map, sequence, frame_id

    def _start_sequence(self):
        if not self._sequence:
            self.get_logger().warn('No waypoint sequence configured.')
            return

        poses = self._build_sequence(self._sequence)
        if not poses:
            self.get_logger().warn('No valid poses to send.')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending {len(poses)} waypoints.')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _build_sequence(self, sequence: List[str]) -> List[PoseStamped]:
        poses: List[PoseStamped] = []
        for name in sequence:
            entry = self._waypoints_map.get(name)
            if entry is None:
                self.get_logger().warn(f'Waypoint "{name}" not found in YAML.')
                continue

            pose = self._entry_to_pose(name, entry)
            if pose is None:
                continue
            poses.append(pose)
        return poses

    def _entry_to_pose(self, name: str, entry: Dict) -> PoseStamped:
        try:
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

            pose_stamped = PoseStamped(
                header=Header(frame_id=self._frame_id),
                pose=Pose(
                    position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=Quaternion(
                        x=float(qx), y=float(qy), z=float(qz), w=float(qw)
                    ),
                ),
            )
            return pose_stamped
        except Exception as exc:
            self.get_logger().error(f'Invalid waypoint "{name}": {exc}')
            return None

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        self._goal_accept = self._goal_handle.accepted
        if not self._goal_accept:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            result = future.result().result
            missed_waypoints = result.missed_waypoints
            if not missed_waypoints:
                self.get_logger().info('All waypoints followed successfully.')
            else:
                self.get_logger().info(f'Waypoints missed: {missed_waypoints}')
        except Exception as exc:
            self.get_logger().error(f'Exception in get_result_callback: {exc}')
        finally:
            self._goal_handle = None
            self._goal_accept = False
            if self.get_parameter('repeat').value:
                self._schedule_repeat()

    def _schedule_repeat(self):
        delay = float(self.get_parameter('repeat_delay_sec').value)

        def _restart():
            if self._repeat_timer is not None:
                self._repeat_timer.cancel()
                self._repeat_timer = None
            self._start_sequence()

        if delay <= 0.0:
            _restart()
        else:
            self._repeat_timer = self.create_timer(delay, _restart)


def main(args=None):
    rclpy.init(args=args)
    node = YamlWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
