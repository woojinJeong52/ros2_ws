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


# main goal 도착 후 전방 정렬 + 후진 + 회전을 수행할 waypoint
APPROACH_BACKUP_ROTATE_WAYPOINTS = {
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
        # Merged LaserScan
        # =========================================================
        # 병합된 /scan 데이터를 사용한다.
        self.declare_parameter('scan_topic', '/scan')

        # /scan 배열의 처음 N개 + 끝 N개를 전방 데이터로 사용한다.
        # 현재 /scan 구조에서는 전방이 배열 양끝에 걸쳐 있음.
        self.declare_parameter('front_edge_sample_count', 5)

        # 배열 맨 앞/맨 뒤에서 몇 개를 건너뛸지.
        self.declare_parameter('front_edge_skip_count', 0)

        # scan이 이 시간보다 오래되면 정렬 중 전진/후진 금지
        self.declare_parameter('scan_stale_timeout_sec', 0.5)

        # 원본 ranges[] 값을 직접 사용하기 위한 코드 내부 유효 거리 기준
        self.declare_parameter('front_min_valid_range', 0.005)
        self.declare_parameter('front_max_valid_range', 5.0)

        # 디버그 로그
        self.declare_parameter('scan_debug', True)
        self.declare_parameter('scan_debug_period_sec', 1.0)

        # =========================================================
        # Front distance alignment
        # =========================================================
        # /scan 전방 edge 데이터 기준 47cm 거리로 정렬
        self.declare_parameter('approach_after_goal', True)
        self.declare_parameter('target_front_distance', 0.47)

        # 허용 오차: 47cm ± 1cm 안에 들어오면 정렬 완료
        self.declare_parameter('target_distance_tolerance', 0.01)

        # 정렬 속도
        self.declare_parameter('approach_speed', 0.08)
        self.declare_parameter('approach_min_speed', 0.05)

        # 목표 거리 근처에서 감속 시작할 범위
        self.declare_parameter('approach_slowdown_distance', 0.15)

        # 정렬 제한 시간
        self.declare_parameter('approach_timeout_sec', 12.0)

        # =========================================================
        # Backup
        # =========================================================
        # 정렬 후 cmd_vel로 20cm 후진
        self.declare_parameter('backup_after_goal', True)
        self.declare_parameter('backup_distance', 0.20)
        self.declare_parameter('backup_speed', 0.08)
        self.declare_parameter('backup_timeout_sec', 5.0)

        # =========================================================
        # Rotation after backup
        # =========================================================
        # 20cm 후진 후 반시계방향 120도 회전
        self.declare_parameter('rotate_after_backup', True)
        self.declare_parameter('rotate_angle_deg', 150.0)

        # rad/s, 양수 방향이 반시계방향
        self.declare_parameter('rotate_angular_speed', 0.5)
        self.declare_parameter('rotate_timeout_sec', 8.0)

        # timer
        self.declare_parameter('motion_timer_period_sec', 0.05)

        # =========================================================
        # Internal states
        # =========================================================
        self._current_index = 0
        self._current_name: Optional[str] = None

        # phase:
        # IDLE -> NAVIGATING -> APPROACHING -> BACKING_UP -> ROTATING -> IDLE
        # DONE / ERROR
        self._phase = 'IDLE'
        self._mission_done = False

        self._nav_goal_handle = None

        self._latest_front_distance: Optional[float] = None
        self._latest_scan_time = None
        self._latest_scan_frame_id: Optional[str] = None
        self._latest_scan_range_min: Optional[float] = None
        self._latest_scan_range_max: Optional[float] = None
        self._last_scan_debug_time = None

        self._last_front_candidates = []
        self._last_front_valid_distances = []
        self._last_front_median: Optional[float] = None

        self._approach_timer = None
        self._approach_start_time = None

        self._backup_timer = None
        self._backup_start_time = None
        self._backup_required_time = 0.0

        self._rotate_timer = None
        self._rotate_start_time = None
        self._rotate_required_time = 0.0

        self._continue_on_miss = bool(
            self.get_parameter('continue_on_miss').value
        )

        self._follow_waypoints_server_timeout_sec = float(
            self.get_parameter('follow_waypoints_server_timeout_sec').value
        )

        self._front_edge_sample_count = int(
            self.get_parameter('front_edge_sample_count').value
        )

        self._front_edge_skip_count = int(
            self.get_parameter('front_edge_skip_count').value
        )

        self._scan_stale_timeout_sec = float(
            self.get_parameter('scan_stale_timeout_sec').value
        )

        self._front_min_valid_range = float(
            self.get_parameter('front_min_valid_range').value
        )

        self._front_max_valid_range = float(
            self.get_parameter('front_max_valid_range').value
        )

        self._scan_debug = bool(
            self.get_parameter('scan_debug').value
        )

        self._scan_debug_period_sec = float(
            self.get_parameter('scan_debug_period_sec').value
        )

        self._approach_after_goal = bool(
            self.get_parameter('approach_after_goal').value
        )

        self._target_front_distance = float(
            self.get_parameter('target_front_distance').value
        )

        self._target_distance_tolerance = float(
            self.get_parameter('target_distance_tolerance').value
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

        self._rotate_after_backup = bool(
            self.get_parameter('rotate_after_backup').value
        )

        self._rotate_angle_deg = float(
            self.get_parameter('rotate_angle_deg').value
        )

        self._rotate_angle_rad = math.radians(self._rotate_angle_deg)

        self._rotate_angular_speed = float(
            self.get_parameter('rotate_angular_speed').value
        )

        self._rotate_timeout_sec = float(
            self.get_parameter('rotate_timeout_sec').value
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
            f'Merged scan topic: {scan_topic}, '
            f'front_edge_sample_count: {self._front_edge_sample_count}, '
            f'front_edge_skip_count: {self._front_edge_skip_count}, '
            f'target_front_distance: {self._target_front_distance:.3f} m, '
            f'tolerance: ±{self._target_distance_tolerance:.3f} m, '
            f'backup_distance: {self._backup_distance:.3f} m, '
            f'rotate_after_backup: {self._rotate_after_backup}, '
            f'rotate_angle: {self._rotate_angle_deg:.1f} deg, '
            f'rotate_speed: {self._rotate_angular_speed:.3f} rad/s'
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
                self._start_front_alignment()
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
            name in APPROACH_BACKUP_ROTATE_WAYPOINTS
            and (
                self._approach_after_goal
                or self._backup_after_goal
                or self._rotate_after_backup
            )
        )

    # =========================================================
    # LaserScan
    # =========================================================
    def _scan_callback(self, msg: LaserScan):
        front_distance = self._get_front_distance(msg)

        self._scan_debug_log(msg, front_distance)

        if front_distance is not None:
            self._latest_front_distance = front_distance
            self._latest_scan_time = self.get_clock().now()
            self._latest_scan_frame_id = msg.header.frame_id
            self._latest_scan_range_min = msg.range_min
            self._latest_scan_range_max = msg.range_max

    def _scan_debug_log(self, msg: LaserScan, front_distance: Optional[float]):
        if not self._scan_debug:
            return

        now = self.get_clock().now()

        if self._last_scan_debug_time is not None:
            elapsed = (
                now - self._last_scan_debug_time
            ).nanoseconds / 1e9

            if elapsed < self._scan_debug_period_sec:
                return

        self._last_scan_debug_time = now

        candidates_str = [
            round(v, 3) if math.isfinite(v) else 'inf'
            for v in self._last_front_candidates
        ]

        valid_str = [
            round(v, 3)
            for v in self._last_front_valid_distances
        ]

        self.get_logger().info(
            f'[SCAN MEDIAN DEBUG] '
            f'frame={msg.header.frame_id}, '
            f'ranges_len={len(msg.ranges)}, '
            f'edge_sample_count={self._front_edge_sample_count}, '
            f'edge_skip_count={self._front_edge_skip_count}, '
            f'front_candidates={candidates_str}, '
            f'valid_sorted={valid_str}, '
            f'median={self._last_front_median}, '
            f'front_distance={front_distance}'
        )

    def _get_front_distance(self, msg: LaserScan) -> Optional[float]:
        if not msg.ranges:
            self._last_front_candidates = []
            self._last_front_valid_distances = []
            self._last_front_median = None
            return None

        ranges = list(msg.ranges)
        total_count = len(ranges)

        sample_count = max(1, self._front_edge_sample_count)
        skip_count = max(0, self._front_edge_skip_count)

        required_count = (sample_count + skip_count) * 2

        if total_count < required_count:
            self._last_front_candidates = []
            self._last_front_valid_distances = []
            self._last_front_median = None
            return None

        front_candidates = []

        # 배열 시작부 N개
        front_candidates.extend(
            ranges[skip_count: skip_count + sample_count]
        )

        # 배열 끝부분 N개
        if skip_count == 0:
            front_candidates.extend(
                ranges[-sample_count:]
            )
        else:
            front_candidates.extend(
                ranges[-skip_count - sample_count: -skip_count]
            )

        valid_distances = []

        for raw_distance in front_candidates:
            if not math.isfinite(raw_distance):
                continue

            if raw_distance < self._front_min_valid_range:
                continue

            if raw_distance > self._front_max_valid_range:
                continue

            valid_distances.append(raw_distance)

        self._last_front_candidates = front_candidates
        self._last_front_valid_distances = list(valid_distances)

        if not valid_distances:
            self._last_front_median = None
            return None

        valid_distances.sort()
        mid = len(valid_distances) // 2

        # median 사용
        if len(valid_distances) % 2 == 1:
            median_distance = valid_distances[mid]
        else:
            median_distance = 0.5 * (
                valid_distances[mid - 1] + valid_distances[mid]
            )

        self._last_front_valid_distances = valid_distances
        self._last_front_median = median_distance

        return median_distance

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
    # Front distance alignment
    # =========================================================
    def _start_front_alignment(self):
        if not self._approach_after_goal:
            self._start_cmd_vel_backup()
            return

        if self._phase != 'NAVIGATING':
            self.get_logger().warn(
                f'Front alignment start ignored because phase is {self._phase}.'
            )
            return

        self._cancel_approach_timer()
        self._publish_zero_velocity()

        self._phase = 'APPROACHING'

        front_distance = self._get_recent_front_distance()

        self.get_logger().info(
            f'[ALIGN START] index={self._current_index}, '
            f'name="{self._current_name}", '
            f'target_distance={self._target_front_distance:.3f} m, '
            f'tolerance=±{self._target_distance_tolerance:.3f} m, '
            f'front_distance={front_distance}, '
            f'scan_frame="{self._latest_scan_frame_id}"'
        )

        self._approach_start_time = self.get_clock().now()

        self._approach_timer = self.create_timer(
            self._motion_timer_period_sec,
            self._front_alignment_timer_callback,
        )

    def _front_alignment_timer_callback(self):
        if self._phase != 'APPROACHING':
            self.get_logger().warn(
                f'Front alignment timer ignored because phase is {self._phase}.'
            )
            self._cancel_approach_timer()
            return

        if self._approach_start_time is None:
            self._finish_front_alignment()
            return

        elapsed = (
            self.get_clock().now() - self._approach_start_time
        ).nanoseconds / 1e9

        front_distance = self._get_recent_front_distance()

        if front_distance is None:
            self._publish_zero_velocity()

            if elapsed >= self._approach_timeout_sec:
                self.get_logger().warn(
                    f'[ALIGN TIMEOUT] valid /scan data unavailable. '
                    f'index={self._current_index}, '
                    f'name="{self._current_name}"'
                )
                self._finish_front_alignment()

            return

        error = front_distance - self._target_front_distance

        if abs(error) <= self._target_distance_tolerance:
            self.get_logger().info(
                f'[ALIGN DONE] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'front_distance={front_distance:.3f} m, '
                f'target={self._target_front_distance:.3f} m'
            )
            self._finish_front_alignment()
            return

        if elapsed >= self._approach_timeout_sec:
            self.get_logger().warn(
                f'[ALIGN TIMEOUT] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'front_distance={front_distance:.3f} m, '
                f'target={self._target_front_distance:.3f} m'
            )
            self._finish_front_alignment()
            return

        cmd = Twist()

        # front_distance > target:
        # 장애물이 멀다 -> 앞으로 접근
        if error > 0.0:
            cmd.linear.x = self._compute_alignment_speed(abs(error))

        # front_distance < target:
        # 너무 가깝다 -> 뒤로 물러남
        else:
            cmd.linear.x = -self._compute_alignment_speed(abs(error))

        self._cmd_vel_pub.publish(cmd)

    def _compute_alignment_speed(self, abs_error: float) -> float:
        if abs_error <= self._target_distance_tolerance:
            return 0.0

        slowdown = max(
            self._approach_slowdown_distance,
            self._target_distance_tolerance + 0.001,
        )

        if abs_error >= slowdown:
            return abs(self._approach_speed)

        ratio = abs_error / slowdown
        speed = abs(self._approach_speed) * ratio

        return max(
            abs(self._approach_min_speed),
            min(speed, abs(self._approach_speed)),
        )

    def _finish_front_alignment(self):
        if self._phase != 'APPROACHING':
            self.get_logger().warn(
                f'Finish front alignment ignored because phase is {self._phase}.'
            )
            return

        self._cancel_approach_timer()
        self._publish_zero_velocity()

        self.get_logger().info(
            f'[ALIGN FINISH] index={self._current_index}, '
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
            self._start_cmd_vel_rotation()
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
            self._start_cmd_vel_rotation()
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

        self._start_cmd_vel_rotation()

    def _cancel_backup_timer(self):
        if self._backup_timer is not None:
            self._backup_timer.cancel()
            self._backup_timer = None

        self._backup_start_time = None
        self._backup_required_time = 0.0

    # =========================================================
    # cmd_vel rotation
    # =========================================================
    def _start_cmd_vel_rotation(self):
        if not self._rotate_after_backup:
            self._advance_to_next()
            return

        if self._phase not in ['BACKING_UP', 'APPROACHING', 'NAVIGATING']:
            self.get_logger().warn(
                f'Rotation start ignored because phase is {self._phase}.'
            )
            return

        self._cancel_rotate_timer()
        self._publish_zero_velocity()

        if self._rotate_angle_rad <= 0.0 or self._rotate_angular_speed <= 0.0:
            self.get_logger().warn(
                'Rotation skipped because rotate angle or angular speed is invalid.'
            )
            self._advance_to_next()
            return

        self._phase = 'ROTATING'

        self._rotate_required_time = (
            abs(self._rotate_angle_rad) / abs(self._rotate_angular_speed)
        )

        self._rotate_start_time = self.get_clock().now()

        self.get_logger().info(
            f'[ROTATE START] index={self._current_index}, '
            f'name="{self._current_name}", '
            f'angle={self._rotate_angle_deg:.1f} deg, '
            f'angular_speed={self._rotate_angular_speed:.3f} rad/s, '
            f'direction=CCW, '
            f'required_time={self._rotate_required_time:.2f} sec'
        )

        self._rotate_timer = self.create_timer(
            self._motion_timer_period_sec,
            self._rotate_timer_callback,
        )

    def _rotate_timer_callback(self):
        if self._phase != 'ROTATING':
            self.get_logger().warn(
                f'Rotate timer ignored because phase is {self._phase}.'
            )
            self._cancel_rotate_timer()
            return

        if self._rotate_start_time is None:
            self._finish_cmd_vel_rotation()
            return

        elapsed = (
            self.get_clock().now() - self._rotate_start_time
        ).nanoseconds / 1e9

        if elapsed >= self._rotate_required_time:
            self.get_logger().info(
                f'[ROTATE DONE] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'elapsed={elapsed:.2f} sec'
            )
            self._finish_cmd_vel_rotation()
            return

        if elapsed >= self._rotate_timeout_sec:
            self.get_logger().warn(
                f'[ROTATE TIMEOUT] index={self._current_index}, '
                f'name="{self._current_name}", '
                f'elapsed={elapsed:.2f} sec'
            )
            self._finish_cmd_vel_rotation()
            return

        cmd = Twist()

        # ROS 기준 angular.z 양수 = 반시계방향 회전
        cmd.angular.z = abs(self._rotate_angular_speed)

        self._cmd_vel_pub.publish(cmd)

    def _finish_cmd_vel_rotation(self):
        if self._phase != 'ROTATING':
            self.get_logger().warn(
                f'Finish rotation ignored because phase is {self._phase}.'
            )
            return

        self._cancel_rotate_timer()
        self._publish_zero_velocity()

        self.get_logger().info(
            f'[ROTATE FINISH] index={self._current_index}, '
            f'name="{self._current_name}"'
        )

        self._advance_to_next()

    def _cancel_rotate_timer(self):
        if self._rotate_timer is not None:
            self._rotate_timer.cancel()
            self._rotate_timer = None

        self._rotate_start_time = None
        self._rotate_required_time = 0.0

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
        self._cancel_rotate_timer()
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
        self._cancel_rotate_timer()
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