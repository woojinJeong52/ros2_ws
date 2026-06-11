from pathlib import Path
from typing import Dict, Optional

import math

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# =========================================================
# Fixed mission sequence
# =========================================================
MISSION_SEQUENCE = [
    'storage_shelf_sub_goal',
    'storage_shelf_goal',
    'workbench_sub_goal',
    'workbench_goal',
    'customer_counter_sub_goal',
    'customer_counter_goal',
    'start_area_sub_goal',
]


# main goal 도착 후 전방 접근 + 후진을 수행할 waypoint
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
        self.declare_parameter('continue_on_miss', False)

        # Nav2 FollowWaypoints
        self.declare_parameter('follow_waypoints_action_name', 'follow_waypoints')
        self.declare_parameter('follow_waypoints_server_timeout_sec', 10.0)

        # cmd_vel
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # =========================================================
        # Front LiDAR
        # =========================================================
        # 중요:
        # /scan은 전방/후방 라이다가 섞일 수 있으므로,
        # 전방 라이다 원본 토픽을 직접 사용한다.
        self.declare_parameter('scan_topic', '/rplidar1/scan')
        self.declare_parameter('front_lidar_frame_id', 'lidar1_link')

        # 라이다 angle 0도 기준으로 전방을 본다.
        # 만약 전방 감지가 반대로 잡히면 180.0으로 바꿔서 테스트.
        self.declare_parameter('front_angle_center_deg', 0.0)

        # 전방 중심 기준 ±10도만 사용
        self.declare_parameter('front_angle_deg', 10.0)

        # scan이 이 시간보다 오래되면 전진 금지
        self.declare_parameter('scan_stale_timeout_sec', 0.5)

        # =========================================================
        # Front approach
        # =========================================================
        # 전방 라이다 기준 장애물과 5cm 간격을 두고 정지
        self.declare_parameter('approach_after_goal', True)
        self.declare_parameter('approach_stop_distance', 0.05)

        # 기본 접근 속도
        self.declare_parameter('approach_speed', 0.03)

        # 정지거리 근처에서는 더 천천히 접근
        self.declare_parameter('approach_min_speed', 0.01)
        self.declare_parameter('approach_slowdown_distance', 0.12)

        # 접근 제한 시간
        self.declare_parameter('approach_timeout_sec', 8.0)

        # =========================================================
        # Backup
        # =========================================================
        # cmd_vel로 20cm 후진
        self.declare_parameter('backup_after_goal', True)
        self.declare_parameter('backup_distance', 0.20)
        self.declare_parameter('backup_speed', 0.08)
        self.declare_parameter('backup_timeout_sec', 5.0)

        # timer
        self.declare_parameter('motion_timer_period_sec', 0.05)

        # =========================================================
        # Internal states
        # =========================================================
        self._current_index = 0
        self._current_name: Optional[str] = None

        # phase:
        # IDLE -> NAVIGATING -> APPROACHING -> BACKING_UP -> IDLE
        # DONE / ERROR
        self._phase = 'IDLE'
        self._mission_done = False

        self._nav_goal_handle = None

        self._latest_front_distance: Optional[float] = None
        self._latest_scan_time = None
        self._latest_scan_range_min: Optional[float] = None
        self._latest_scan_range_max: Optional[float] = None

        self._approach_timer = None
        self._approach_start_time = None

        self._backup_timer = None
        self._backup_start_time = None
        self._backup_required_time = 0.0

        self._continue_on_miss = bool(
            self.get_parameter('continue_on_miss').value
        )

        self._follow_waypoints_server_timeout_sec = float(
            self.get_parameter('follow_waypoints_server_timeout_sec').value
        )

        self._front_lidar_frame_id = str(
            self.get_parameter('front_lidar_frame_id').value
        )

        self._front_angle_center_rad = math.radians(
            float(self.get_parameter('front_angle_center_deg').value)
        )

        self._front_angle_rad = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )

        self._scan_stale_timeout_sec = float(
            self.get_parameter('scan_stale_timeout_sec').value
        )

        self._approach_after_goal = bool(
            self.get_parameter('approach_after_goal').value
        )

        self._approach_stop_distance = float(
            self.get_parameter('approach_stop_distance').value
        )

        self._approach_speed = float(
            self.get_parameter('approach_speed').value
        )

        self._approach_min_speed = float(
            self.get_parameter('approach_min_speed').value
        )

        self._approach_slowdown_distance = float(
            self.get_parameter('approach_slowdown_distance').value
        )

        self._approach_timeout_sec = float(
            self.get_parameter('approach_timeout_sec').value
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

        self._backup_timeout_sec = float(
            self.get_parameter('backup_timeout_sec').value
        )

        self._motion_timer_period_sec = float(
            self.get_parameter('motion_timer_period_sec').value
        )

        # =========================================================
        # ROS interfaces
        # =========================================================
        follow_waypoints_action_name = self.get_parameter(
            'follow_waypoints_action_name'
        ).value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            follow_waypoints_action_name,
        )

        self._cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self._scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        # =========================================================
        # Load waypoints
        # =========================================================
        self._waypoints_map, self._frame_id = self._load_waypoints()

        self.get_logger().info(
            f'Mission sequence fixed: {MISSION_SEQUENCE}'
        )

        self.get_logger().info(
            f'Front LiDAR topic: {scan_topic}, '
            f'expected frame_id: {self._front_lidar_frame_id}, '
            f'approach_stop_distance: {self._approach_stop_distance:.3f} m'
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
        if self._mission_done:
            return

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
            self._phase = 'ERROR'
            return

        pose = self._entry_to_pose(name, entry)

        if pose is None:
            self.get_logger().error(
                f'Waypoint "{name}" invalid. Mission stopped.'
            )
            self._publish_zero_velocity()
            self._phase = 'ERROR'
            return

        if not self._action_client.wait_for_server(
            timeout_sec=self._follow_waypoints_server_timeout_sec
        ):
            self.get_logger().error(
                'FollowWaypoints action server unavailable. Mission stopped.'
            )
            self._publish_zero_velocity()
            self._phase = 'ERROR'
            return

        self._current_name = name
        self._phase = 'NAVIGATING'

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]

        self.get_logger().info(
            f'[NAV START] index={self._current_index}, '
            f'name="{name}", phase={self._phase}'
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
            self._phase = 'ERROR'
            return

        if not self._nav_goal_handle.accepted:
            self.get_logger().warn(
                f'Navigation goal rejected: {self._current_name}'
            )

            if self._continue_on_miss:
                self._advance_to_next()
            else:
                self._publish_zero_velocity()
                self._phase = 'ERROR'

            return

        result_future = self._nav_goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        if self._phase != 'NAVIGATING':
            self.get_logger().warn(
                f'Navigation result ignored because phase is {self._phase}. '
                f'current_name={self._current_name}, index={self._current_index}'
            )
            return

        try:
            result = future.result().result

            if result.missed_waypoints:
                self.get_logger().warn(
                    f'Waypoint "{self._current_name}" missed: '
                    f'{result.missed_waypoints}'
                )

                if self._continue_on_miss:
                    self._advance_to_next()
                    return

                self.get_logger().warn(
                    'Mission stopped due to missed waypoint.'
                )
                self._publish_zero_velocity()
                self._phase = 'ERROR'
                return

            self.get_logger().info(
                f'[NAV DONE] index={self._current_index}, '
                f'name="{self._current_name}"'
            )

            if self._should_post_process(self._current_name):
                self._start_front_approach()
            else:
                self._advance_to_next()

        except Exception as exc:
            self.get_logger().error(
                f'Exception in navigation result callback: {exc}'
            )
            self._publish_zero_velocity()
            self._phase = 'ERROR'

        finally:
            self._nav_goal_handle = None

    def _should_post_process(self, name: Optional[str]) -> bool:
        return (
            name in APPROACH_AND_BACKUP_WAYPOINTS
            and (
                self._approach_after_goal
                or self._backup_after_goal
            )
        )

    # =========================================================
    # LaserScan
    # =========================================================
    def _scan_callback(self, msg: LaserScan):
        # 전방 라이다 frame만 사용
        if self._front_lidar_frame_id:
            msg_frame = msg.header.frame_id.lstrip('/')
            expected_frame = self._front_lidar_frame_id.lstrip('/')

            if msg_frame != expected_frame:
                return

        front_distance = self._get_front_distance(msg)

        if front_distance is not None:
            self._latest_front_distance = front_distance
            self._latest_scan_time = self.get_clock().now()
            self._latest_scan_range_min = msg.range_min
            self._latest_scan_range_max = msg.range_max

    def _get_front_distance(self, msg: LaserScan) -> Optional[float]:
        if not msg.ranges or msg.angle_increment == 0.0:
            return None

        distances = []

        for index, raw_distance in enumerate(msg.ranges):
            if not math.isfinite(raw_distance):
                continue

            angle = msg.angle_min + (index * msg.angle_increment)

            # angle wrap 처리
            angle_error = math.atan2(
                math.sin(angle - self._front_angle_center_rad),
                math.cos(angle - self._front_angle_center_rad),
            )

            if abs(angle_error) > self._front_angle_rad:
                continue

            if raw_distance < msg.range_min or raw_distance > msg.range_max:
                continue

            distances.append(raw_distance)

        if not distances:
            return None

        # 전방 ±front_angle_deg 범위에서 가장 가까운 장애물 거리
        return min(distances)

    def _get_recent_front_distance(self) -> Optional[float]:
        if self._latest_scan_time is None:
            return None

        age = (
            self.get_clock().now() - self._latest_scan_time
        ).nanoseconds / 1e9

        if age > self._scan_stale_timeout_sec:
            return None

        return self._latest_front_distance

    # =========================================================
    # Front approach
    # =========================================================
    def _start_front_approach(self):
        if not self._approach_after_goal:
            self._start_cmd_vel_backup()
            return

        if self._phase != 'NAVIGATING':
            self.get_logger().warn(
                f'Approach start ignored because phase is {self._phase}.'
            )
            return

        self._cancel_approach_timer()
        self._publish_zero_velocity()

        self._phase = 'APPROACHING'

        front_distance = self._get_recent_front_distance()

        self.get_logger().info(
            f'[APPROACH START] index={self._current_index}, '
            f'name="{self._current_name}", '
            f'stop_distance={self._approach_stop_distance:.3f} m, '
            f'front_distance={front_distance}, '
            f'front_lidar_frame="{self._front_lidar_frame_id}"'
        )

        if (
            self._latest_scan_range_min is not None
            and self._approach_stop_distance < self._latest_scan_range_min
        ):
            self.get_logger().warn(
                f'approach_stop_distance={self._approach_stop_distance:.3f} m '
                f'is smaller than scan range_min={self._latest_scan_range_min:.3f} m. '
                f'LiDAR may not reliably measure the requested stop distance.'
            )

        self._approach_start_time = self.get_clock().now()

        self._approach_timer = self.create_timer(
            self._motion_timer_period_sec,
            self._approach_timer_callback,
        )

    def _approach_timer_callback(self):
        if self._phase != 'APPROACHING':
            self.get_logger().warn(
                f'Approach timer ignored because phase is {self._phase}.'
            )
            self._cancel_approach_timer()
            return

        if self._approach_start_time is None:
            self._finish_front_approach()
            return

        elapsed = (
            self.get_clock().now() - self._approach_start_time
        ).nanoseconds / 1e9

        front_distance = self._get_recent_front_distance()

        # scan이 없으면 절대 전진하지 않는다.
        if front_distance is None:
            self._publish_zero_velocity()

            if elapsed >= self._approach_timeout_sec:
                self.get_logger().warn(
                    f'[APPROACH TIMEOUT] valid front LiDAR scan unavailable. '
                    f'index={self._current_index}, '
                    f'name="{self._current_name}"'
                )
                self._finish_front_approach()

            return

        # 전방 라이다 기준 장애물과 5cm 이하가 되면 정지
        if front_distance <= self._approach_stop_distance:
            self.get_logger().info(
                f'[APPROACH DONE] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'front_distance={front_distance:.3f} m'
            )
            self._finish_front_approach()
            return

        # timeout이면 접근 종료 후 후진
        if elapsed >= self._approach_timeout_sec:
            self.get_logger().warn(
                f'[APPROACH TIMEOUT] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'front_distance={front_distance:.3f} m'
            )
            self._finish_front_approach()
            return

        cmd = Twist()
        cmd.linear.x = self._compute_approach_speed(front_distance)
        self._cmd_vel_pub.publish(cmd)

    def _compute_approach_speed(self, front_distance: float) -> float:
        # 정지 거리 근처에서는 감속한다.
        stop = self._approach_stop_distance
        slowdown = max(self._approach_slowdown_distance, stop + 0.001)

        if front_distance <= stop:
            return 0.0

        if front_distance >= slowdown:
            return abs(self._approach_speed)

        ratio = (front_distance - stop) / (slowdown - stop)
        speed = abs(self._approach_speed) * ratio

        return max(abs(self._approach_min_speed), min(speed, abs(self._approach_speed)))

    def _finish_front_approach(self):
        if self._phase != 'APPROACHING':
            self.get_logger().warn(
                f'Finish approach ignored because phase is {self._phase}.'
            )
            return

        self._cancel_approach_timer()
        self._publish_zero_velocity()

        self.get_logger().info(
            f'[APPROACH FINISH] index={self._current_index}, '
            f'name="{self._current_name}"'
        )

        self._start_cmd_vel_backup()

    def _cancel_approach_timer(self):
        if self._approach_timer is not None:
            self._approach_timer.cancel()
            self._approach_timer = None

        self._approach_start_time = None

    # =========================================================
    # cmd_vel backup
    # =========================================================
    def _start_cmd_vel_backup(self):
        if not self._backup_after_goal:
            self._advance_to_next()
            return

        if self._phase not in ['APPROACHING', 'NAVIGATING']:
            self.get_logger().warn(
                f'Backup start ignored because phase is {self._phase}.'
            )
            return

        self._cancel_backup_timer()
        self._publish_zero_velocity()

        if self._backup_distance <= 0.0 or self._backup_speed <= 0.0:
            self.get_logger().warn(
                'Backup skipped because backup distance or speed is invalid.'
            )
            self._advance_to_next()
            return

        self._phase = 'BACKING_UP'

        self._backup_required_time = (
            abs(self._backup_distance) / abs(self._backup_speed)
        )

        self._backup_start_time = self.get_clock().now()

        self.get_logger().info(
            f'[BACKUP START] index={self._current_index}, '
            f'name="{self._current_name}", '
            f'distance={self._backup_distance:.3f} m, '
            f'speed={self._backup_speed:.3f} m/s, '
            f'required_time={self._backup_required_time:.2f} sec'
        )

        self._backup_timer = self.create_timer(
            self._motion_timer_period_sec,
            self._backup_timer_callback,
        )

    def _backup_timer_callback(self):
        if self._phase != 'BACKING_UP':
            self.get_logger().warn(
                f'Backup timer ignored because phase is {self._phase}.'
            )
            self._cancel_backup_timer()
            return

        if self._backup_start_time is None:
            self._finish_cmd_vel_backup()
            return

        elapsed = (
            self.get_clock().now() - self._backup_start_time
        ).nanoseconds / 1e9

        if elapsed >= self._backup_required_time:
            self.get_logger().info(
                f'[BACKUP DONE] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'elapsed={elapsed:.2f} sec'
            )
            self._finish_cmd_vel_backup()
            return

        if elapsed >= self._backup_timeout_sec:
            self.get_logger().warn(
                f'[BACKUP TIMEOUT] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'elapsed={elapsed:.2f} sec'
            )
            self._finish_cmd_vel_backup()
            return

        cmd = Twist()
        cmd.linear.x = -abs(self._backup_speed)
        self._cmd_vel_pub.publish(cmd)

    def _finish_cmd_vel_backup(self):
        if self._phase != 'BACKING_UP':
            self.get_logger().warn(
                f'Finish backup ignored because phase is {self._phase}.'
            )
            return

        self._cancel_backup_timer()
        self._publish_zero_velocity()

        self.get_logger().info(
            f'[BACKUP FINISH] index={self._current_index}, '
            f'name="{self._current_name}"'
        )

        self._advance_to_next()

    def _cancel_backup_timer(self):
        if self._backup_timer is not None:
            self._backup_timer.cancel()
            self._backup_timer = None

        self._backup_start_time = None
        self._backup_required_time = 0.0

    # =========================================================
    # Mission control
    # =========================================================
    def _advance_to_next(self):
        if self._mission_done:
            return

        prev_index = self._current_index
        prev_name = self._current_name

        self._current_index += 1

        self.get_logger().info(
            f'[ADVANCE] prev_index={prev_index}, '
            f'prev_name="{prev_name}" '
            f'-> next_index={self._current_index}'
        )

        self._phase = 'IDLE'

        self._send_current_goal()

    def _finish_mission(self):
        if self._mission_done:
            return

        self._mission_done = True
        self._phase = 'DONE'

        self._cancel_approach_timer()
        self._cancel_backup_timer()
        self._publish_zero_velocity()

        self.get_logger().info(
            'Mission complete. Final waypoint was start_area_sub_goal. '
            'Node remains alive.'
        )

    def _publish_zero_velocity(self):
        self._cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        self._cancel_approach_timer()
        self._cancel_backup_timer()
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