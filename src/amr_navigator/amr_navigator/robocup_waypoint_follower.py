from pathlib import Path
from typing import Dict, Optional

import math

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav2_msgs.action import BackUp, FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# =========================================================
# Fixed mission sequence
# =========================================================
# 반드시 아래 순서로만 동작한다.
# 1. storage_shelf_sub_goal
# 2. storage_shelf_goal
#    - 전방 3cm 접근
#    - 20cm 후진
# 3. workbench_sub_goal
# 4. workbench_goal
#    - 전방 3cm 접근
#    - 20cm 후진
# 5. customer_counter_sub_goal
# 6. customer_counter_goal
#    - 전방 3cm 접근
#    - 20cm 후진
# 7. start_area_sub_goal
# 8. 종료
MISSION_SEQUENCE = [
    'storage_shelf_sub_goal',
    'storage_shelf_goal',
    'workbench_sub_goal',
    'workbench_goal',
    'customer_counter_sub_goal',
    'customer_counter_goal',
    'start_area_sub_goal',
]


# main goal 도착 후 전방 접근과 후진을 수행할 waypoint
APPROACH_AND_BACKUP_WAYPOINTS = {
    'storage_shelf_goal',
    'workbench_goal',
    'customer_counter_goal',
}


class RobocupWaypointFollower(Node):
    def __init__(self):
        super().__init__('robocup_waypoint_follower')

        default_yaml = str(
            Path(get_package_share_directory('amr_navigator'))
            / 'params'
            / 'waypoints_robocup.yaml'
        )

        # =========================================================
        # Parameters
        # =========================================================
        self.declare_parameter('waypoints_file', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)

        # waypoint 실패 시 기본적으로 정지
        self.declare_parameter('continue_on_miss', False)

        # Nav2 FollowWaypoints
        self.declare_parameter('follow_waypoints_action_name', 'follow_waypoints')
        self.declare_parameter('follow_waypoints_server_timeout_sec', 10.0)

        # 전방 접근
        self.declare_parameter('approach_after_goal', True)
        self.declare_parameter('approach_target_distance', 0.03)
        self.declare_parameter('approach_speed', 0.03)
        self.declare_parameter('approach_timeout_sec', 8.0)
        self.declare_parameter('approach_front_angle_deg', 10.0)
        self.declare_parameter('approach_scan_topic', '/scan')
        self.declare_parameter('approach_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('approach_timer_period_sec', 0.05)

        # 후진
        self.declare_parameter('backup_after_goal', True)
        self.declare_parameter('backup_distance', 0.20)
        self.declare_parameter('backup_speed', 0.08)
        self.declare_parameter('backup_time_allowance_sec', 10)
        self.declare_parameter('backup_action_name', 'backup')
        self.declare_parameter('backup_server_timeout_sec', 5.0)

        # =========================================================
        # Internal states
        # =========================================================
        self._current_index = 0
        self._current_name: Optional[str] = None

        self._nav_goal_handle = None
        self._backup_goal_handle = None

        self._approach_timer = None
        self._approach_start_time = None
        self._latest_front_distance: Optional[float] = None

        self._continue_on_miss = bool(
            self.get_parameter('continue_on_miss').value
        )

        self._follow_waypoints_server_timeout_sec = float(
            self.get_parameter('follow_waypoints_server_timeout_sec').value
        )

        self._approach_after_goal = bool(
            self.get_parameter('approach_after_goal').value
        )
        self._approach_target_distance = float(
            self.get_parameter('approach_target_distance').value
        )
        self._approach_speed = float(
            self.get_parameter('approach_speed').value
        )
        self._approach_timeout_sec = float(
            self.get_parameter('approach_timeout_sec').value
        )
        self._approach_front_angle_rad = math.radians(
            float(self.get_parameter('approach_front_angle_deg').value)
        )
        self._approach_timer_period_sec = float(
            self.get_parameter('approach_timer_period_sec').value
        )

        self._backup_after_goal = bool(
            self.get_parameter('backup_after_goal').value
        )
        self._backup_distance = float(
            self.get_parameter('backup_distance').value
        )
        self._backup_speed = float(
            self.get_parameter('backup_speed').value
        )
        self._backup_time_allowance_sec = int(
            self.get_parameter('backup_time_allowance_sec').value
        )
        self._backup_server_timeout_sec = float(
            self.get_parameter('backup_server_timeout_sec').value
        )

        # =========================================================
        # Action clients / publishers / subscribers
        # =========================================================
        follow_waypoints_action_name = self.get_parameter(
            'follow_waypoints_action_name'
        ).value

        backup_action_name = self.get_parameter(
            'backup_action_name'
        ).value

        scan_topic = self.get_parameter(
            'approach_scan_topic'
        ).value

        cmd_vel_topic = self.get_parameter(
            'approach_cmd_vel_topic'
        ).value

        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            follow_waypoints_action_name,
        )

        self._backup_client = ActionClient(
            self,
            BackUp,
            backup_action_name,
        )

        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        # =========================================================
        # Load waypoints
        # =========================================================
        self._waypoints_map, self._frame_id = self._load_waypoints()

        self.get_logger().info(
            f'Mission sequence fixed: {MISSION_SEQUENCE}'
        )

        if self.get_parameter('auto_start').value:
            self._send_current_goal()
        else:
            self.get_logger().info(
                'auto_start is false; waiting for manual start.'
            )

    # =========================================================
    # Waypoint loading
    # =========================================================
    def _load_waypoints(self):
        path = Path(
            str(self.get_parameter('waypoints_file').value)
        ).expanduser()

        frame_id = self.get_parameter('frame_id').value

        if not path.exists():
            self.get_logger().error(f'Waypoints file not found: {path}')
            return {}, frame_id

        try:
            with path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            return {}, frame_id

        if isinstance(data, dict) and data.get('frame_id'):
            frame_id = data.get('frame_id')

        waypoints_map = data.get('waypoints', {}) if isinstance(data, dict) else {}

        missing = [
            name for name in MISSION_SEQUENCE
            if name not in waypoints_map
        ]

        if missing:
            self.get_logger().error(
                f'Missing required mission waypoints: {missing}'
            )
        else:
            self.get_logger().info(
                'All required mission waypoints loaded.'
            )

        # YAML 안에 sequence가 있어도 사용하지 않는다.
        if isinstance(data, dict) and data.get('sequence'):
            self.get_logger().info(
                'YAML sequence detected but ignored. '
                'Using fixed MISSION_SEQUENCE in code.'
            )

        return waypoints_map, frame_id

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
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self._frame_id,
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

    # =========================================================
    # Navigation
    # =========================================================
    def _send_current_goal(self):
        if self._current_index >= len(MISSION_SEQUENCE):
            self._finish_mission()
            return

        name = MISSION_SEQUENCE[self._current_index]
        entry = self._waypoints_map.get(name)

        if entry is None:
            self.get_logger().error(
                f'Waypoint "{name}" not found. Mission stopped.'
            )
            self._publish_zero_velocity()
            return

        pose = self._entry_to_pose(name, entry)

        if pose is None:
            self.get_logger().error(
                f'Waypoint "{name}" invalid. Mission stopped.'
            )
            self._publish_zero_velocity()
            return

        if not self._action_client.wait_for_server(
            timeout_sec=self._follow_waypoints_server_timeout_sec
        ):
            self.get_logger().error(
                'FollowWaypoints action server unavailable. Mission stopped.'
            )
            self._publish_zero_velocity()
            return

        self._current_name = name

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]

        self.get_logger().info(
            f'Navigating to "{name}" '
            f'({self._current_index + 1}/{len(MISSION_SEQUENCE)})'
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            self._nav_goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Failed to send navigation goal: {exc}'
            )
            self._publish_zero_velocity()
            return

        if not self._nav_goal_handle.accepted:
            self.get_logger().warn(
                f'Navigation goal rejected: {self._current_name}'
            )

            if self._continue_on_miss:
                self._advance_to_next()
            else:
                self._publish_zero_velocity()

            return

        result_future = self._nav_goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            result = future.result().result

            if result.missed_waypoints:
                self.get_logger().warn(
                    f'Waypoint "{self._current_name}" missed: '
                    f'{result.missed_waypoints}'
                )

                if not self._continue_on_miss:
                    self.get_logger().warn('Mission stopped due to missed waypoint.')
                    self._publish_zero_velocity()
                    return

            self.get_logger().info(
                f'Arrived at "{self._current_name}"'
            )

            if self._should_approach_and_backup(self._current_name):
                self._start_front_approach()
            else:
                self._advance_to_next()

        except Exception as exc:
            self.get_logger().error(
                f'Exception in navigation result callback: {exc}'
            )
            self._publish_zero_velocity()

        finally:
            self._nav_goal_handle = None

    # =========================================================
    # Approach
    # =========================================================
    def _should_approach_and_backup(self, name: Optional[str]) -> bool:
        return (
            name in APPROACH_AND_BACKUP_WAYPOINTS
            and self._approach_after_goal
        )

    def _scan_callback(self, msg: LaserScan):
        front_distance = self._get_front_distance(msg)

        if front_distance is not None:
            self._latest_front_distance = front_distance

    def _get_front_distance(self, msg: LaserScan) -> Optional[float]:
        if not msg.ranges or msg.angle_increment == 0.0:
            return None

        distances = []

        for index, raw_distance in enumerate(msg.ranges):
            if not math.isfinite(raw_distance):
                continue

            angle = msg.angle_min + (index * msg.angle_increment)

            if abs(angle) > self._approach_front_angle_rad:
                continue

            if raw_distance < msg.range_min or raw_distance > msg.range_max:
                continue

            distances.append(raw_distance)

        if not distances:
            return None

        return min(distances)

    def _start_front_approach(self):
        distance = self._latest_front_distance

        if distance is not None and distance <= self._approach_target_distance:
            self.get_logger().info(
                f'Front distance already {distance:.3f} m. '
                f'Skipping approach after "{self._current_name}".'
            )
            self._send_backup_goal()
            return

        self._approach_start_time = self.get_clock().now()

        self.get_logger().info(
            f'Front approach started after "{self._current_name}". '
            f'Target distance: {self._approach_target_distance:.3f} m'
        )

        self._approach_timer = self.create_timer(
            self._approach_timer_period_sec,
            self._approach_timer_callback,
        )

    def _approach_timer_callback(self):
        if self._approach_start_time is None:
            self._finish_front_approach()
            return

        elapsed = (
            self.get_clock().now() - self._approach_start_time
        ).nanoseconds / 1e9

        distance = self._latest_front_distance

        if distance is not None and distance <= self._approach_target_distance:
            self.get_logger().info(
                f'Front approach complete after "{self._current_name}". '
                f'Front distance: {distance:.3f} m'
            )
            self._finish_front_approach()
            return

        if elapsed >= self._approach_timeout_sec:
            if distance is None:
                self.get_logger().warn(
                    f'Front approach timed out after "{self._current_name}" '
                    f'without valid scan data.'
                )
            else:
                self.get_logger().warn(
                    f'Front approach timed out after "{self._current_name}". '
                    f'Current front distance: {distance:.3f} m'
                )

            self._finish_front_approach()
            return

        if distance is None:
            self._publish_zero_velocity()
            return

        cmd = Twist()
        cmd.linear.x = abs(self._approach_speed)
        self._cmd_vel_pub.publish(cmd)

    def _finish_front_approach(self):
        if self._approach_timer is not None:
            self._approach_timer.cancel()
            self._approach_timer = None

        self._approach_start_time = None
        self._publish_zero_velocity()

        self._send_backup_goal()

    # =========================================================
    # BackUp
    # =========================================================
    def _send_backup_goal(self):
        if not self._backup_after_goal:
            self._advance_to_next()
            return

        if self._current_name not in APPROACH_AND_BACKUP_WAYPOINTS:
            self._advance_to_next()
            return

        if not self._backup_client.wait_for_server(
            timeout_sec=self._backup_server_timeout_sec
        ):
            self.get_logger().warn(
                'BackUp action server unavailable. '
                'Continuing to next waypoint.'
            )
            self._advance_to_next()
            return

        goal_msg = BackUp.Goal()
        goal_msg.target = Point(
            x=-abs(self._backup_distance),
            y=0.0,
            z=0.0,
        )
        goal_msg.speed = abs(self._backup_speed)
        goal_msg.time_allowance = Duration(
            sec=self._backup_time_allowance_sec
        )

        self.get_logger().info(
            f'Backing up {abs(self._backup_distance):.2f} m '
            f'after "{self._current_name}"'
        )

        backup_future = self._backup_client.send_goal_async(goal_msg)
        backup_future.add_done_callback(self._backup_response_callback)

    def _backup_response_callback(self, future):
        try:
            self._backup_goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Failed to send BackUp goal: {exc}'
            )
            self._advance_to_next()
            return

        if not self._backup_goal_handle.accepted:
            self.get_logger().warn(
                'BackUp goal rejected. Continuing to next waypoint.'
            )
            self._advance_to_next()
            return

        result_future = self._backup_goal_handle.get_result_async()
        result_future.add_done_callback(self._backup_result_callback)

    def _backup_result_callback(self, future):
        try:
            future.result()

            self.get_logger().info(
                f'BackUp complete after "{self._current_name}"'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Exception in BackUp result callback: {exc}'
            )

        finally:
            self._backup_goal_handle = None
            self._advance_to_next()

    # =========================================================
    # Mission control
    # =========================================================
    def _advance_to_next(self):
        self._current_index += 1
        self._send_current_goal()

    def _finish_mission(self):
        self._publish_zero_velocity()
        self.get_logger().info(
            'Mission complete. Final waypoint was start_area_sub_goal. Node remains alive.'
        )

    def _publish_zero_velocity(self):
        self._cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        if self._approach_timer is not None:
            self._approach_timer.cancel()
            self._approach_timer = None

        self._publish_zero_velocity()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RobocupWaypointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()