from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header


ROBOCUP_SEQUENCE = [
    'storage_shelf_sub_goal',
    'storage_shelf_goal',
    'workbench_sub_goal',
    'workbench_goal',
    'customer_counter_sub_goal',
    'customer_counter_goal',
    'start_area_sub_goal',
    'start_area_goal',
]


class RobocupWaypointFollower(Node):
    def __init__(self):
        super().__init__('robocup_waypoint_follower')

        default_yaml = str(
            Path(get_package_share_directory('amr_navigator'))
            / 'params'
            / 'waypoints_robocup.yaml'
        )
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('continue_on_miss', False)

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._goal_handle = None
        self._current_index = 0
        self._current_name: Optional[str] = None
        self._continue_on_miss = bool(self.get_parameter('continue_on_miss').value)

        self._waypoints_map, self._sequence, self._frame_id = self._load_waypoints()

        if self.get_parameter('auto_start').value:
            self._send_current_goal()
        else:
            self.get_logger().info('auto_start is false; waiting for manual start.')

    def _load_waypoints(self):
        path = Path(str(self.get_parameter('waypoints_file').value)).expanduser()
        frame_id = self.get_parameter('frame_id').value

        if not path.exists():
            self.get_logger().error(f'Waypoints file not found: {path}')
            return {}, ROBOCUP_SEQUENCE, frame_id

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            return {}, ROBOCUP_SEQUENCE, frame_id

        if isinstance(data, dict) and data.get('frame_id'):
            frame_id = data.get('frame_id')

        waypoints_map = data.get('waypoints', {}) if isinstance(data, dict) else {}
        sequence = data.get('sequence', ROBOCUP_SEQUENCE) if isinstance(data, dict) else ROBOCUP_SEQUENCE
        missing = [name for name in sequence if name not in waypoints_map]
        if missing:
            self.get_logger().error(f'Missing RoboCup waypoints: {missing}')

        return waypoints_map, sequence, frame_id

    def _send_current_goal(self):
        if self._current_index >= len(self._sequence):
            self.get_logger().info('RoboCup waypoint sequence complete.')
            return

        name = self._sequence[self._current_index]
        entry = self._waypoints_map.get(name)
        if entry is None:
            self.get_logger().warn(f'Waypoint "{name}" not found; stopping sequence.')
            return

        pose = self._entry_to_pose(name, entry)
        if pose is None:
            self.get_logger().warn(f'Waypoint "{name}" invalid; stopping sequence.')
            return

        self._current_name = name
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]

        self._action_client.wait_for_server()
        self.get_logger().info(
            f'Sending RoboCup waypoint "{name}" ({self._current_index + 1}/{len(self._sequence)})'
        )
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _entry_to_pose(self, name: str, entry: Dict) -> Optional[PoseStamped]:
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

            return PoseStamped(
                header=Header(frame_id=self._frame_id),
                pose=Pose(
                    position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=Quaternion(
                        x=float(qx), y=float(qy), z=float(qz), w=float(qw)
                    ),
                ),
            )
        except Exception as exc:
            self.get_logger().error(f'Invalid waypoint "{name}": {exc}')
            return None

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected: {self._current_name}')
            if self._continue_on_miss:
                self._advance_to_next()
            return

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            result = future.result().result
            if result.missed_waypoints:
                self.get_logger().warn(
                    f'Waypoint "{self._current_name}" missed: {result.missed_waypoints}'
                )
                if not self._continue_on_miss:
                    return

            self._advance_to_next()
        except Exception as exc:
            self.get_logger().error(f'Exception in result callback: {exc}')
        finally:
            self._goal_handle = None

    def _advance_to_next(self):
        self._current_index += 1
        self._send_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = RobocupWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
