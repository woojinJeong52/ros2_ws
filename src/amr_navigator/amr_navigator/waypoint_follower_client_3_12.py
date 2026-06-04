import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from action_msgs.msg import GoalStatus

import yaml
from pathlib import Path

DEFAULT_WAYPOINTS_YAML = "/home/amr2/ros2_ws/src/amr_navigator/params/waypoints.yaml"


class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')

        # params
        self.declare_parameter('waypoints_yaml', '')
        self.declare_parameter('autostart', True)
        self.declare_parameter('loop_delay_sec', 0.2)

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # optional: control center I/O
        self.control_from_center = self.create_subscription(
            String, '/control_from_center', self.control_from_center_callback, 10
        )
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)

        # state
        self.goal_handle = None
        self.goal_accept = False
        self._sending_goal = False  # send_goal_async 중복 방지
        self._running = False

        self.frame_id = 'map'
        self.ws1 = None
        self.ws2 = None

        # next target: 1이면 ws1로, 2이면 ws2로
        # 시작은 "1 -> 2"를 원하니까 next_target = 2로 두고 ws1에서 출발했다고 가정
        self._next_target = 2

        # one-shot timer handle (중복 생성 방지)
        self._next_goal_timer = None

        # load yaml
        yaml_path = self.get_parameter('waypoints_yaml').value
        if not yaml_path:
            yaml_path = DEFAULT_WAYPOINTS_YAML

        self.get_logger().info(f'Loading waypoints yaml: {yaml_path}')
        self.load_waypoints_yaml(yaml_path)

        # autostart
        if bool(self.get_parameter('autostart').value):
            self.start_loop()

    # --------------------
    # YAML load
    # --------------------
    def load_waypoints_yaml(self, yaml_path: str):
        p = Path(yaml_path)
        if not p.exists():
            raise FileNotFoundError(f'YAML not found: {yaml_path}')

        with p.open('r') as f:
            cfg = yaml.safe_load(f)

        self.frame_id = cfg.get('frame_id', 'map')
        wps = cfg.get('waypoints', {})

        if 'work_station1' not in wps or 'work_station2' not in wps:
            raise KeyError("YAML must contain waypoints.work_station1 and waypoints.work_station2")

        self.ws1 = self.pose_from_dict(wps['work_station1'], self.frame_id)
        self.ws2 = self.pose_from_dict(wps['work_station2'], self.frame_id)

        self.get_logger().info(
            f'Loaded work_station1: ({self.ws1.pose.position.x:.3f}, {self.ws1.pose.position.y:.3f})'
        )
        self.get_logger().info(
            f'Loaded work_station2: ({self.ws2.pose.position.x:.3f}, {self.ws2.pose.position.y:.3f})'
        )

    def pose_from_dict(self, d: dict, frame_id: str) -> PoseStamped:
        pos = d['position']
        ori = d['orientation']
        return PoseStamped(
            header=Header(frame_id=frame_id),
            pose=Pose(
                position=Point(x=float(pos['x']), y=float(pos['y']), z=float(pos.get('z', 0.0))),
                orientation=Quaternion(
                    x=float(ori.get('x', 0.0)),
                    y=float(ori.get('y', 0.0)),
                    z=float(ori.get('z', 0.0)),
                    w=float(ori.get('w', 1.0)),
                ),
            ),
        )

    # --------------------
    # Loop control
    # --------------------
    def start_loop(self):
        if self.ws1 is None or self.ws2 is None:
            self.get_logger().error('Waypoints not loaded. Cannot start.')
            return
        if self._running:
            self.get_logger().warn('Already running.')
            return

        self._running = True
        self.goal_handle = None
        self.goal_accept = False
        self._sending_goal = False

        self.get_logger().info('=== Start ping-pong: ws1 <-> ws2 (arrive -> plan next) ===')
        msg = String()
        msg.data = 'driving_start'
        self.control_from_amr.publish(msg)

        # 첫 goal 발행
        self.send_next_goal_now()

    def stop_loop(self):
        self._running = False
        self.get_logger().info('=== Stop loop ===')
        self._cancel_next_goal_timer()
        self.cancel_goal()

    def _cancel_next_goal_timer(self):
        if self._next_goal_timer is not None:
            try:
                self._next_goal_timer.cancel()
            except Exception:
                pass
            self._next_goal_timer = None

    def send_next_goal_now(self):
        if not self._running:
            return
        if self._sending_goal:
            self.get_logger().warn('send_next_goal_now blocked: already sending a goal.')
            return
        if self.goal_accept and self.goal_handle is not None:
            # goal 수행 중이면 여기서 중복 전송하면 preempt로 경로가 깜빡임
            self.get_logger().warn('send_next_goal_now blocked: goal is still active.')
            return

        # 목표는 "도착지 1개"만 보내는 게 가장 안정적
        if self._next_target == 1:
            target = self.ws1
            self.get_logger().info('[GOAL] -> work_station1')
        else:
            target = self.ws2
            self.get_logger().info('[GOAL] -> work_station2')

        self.send_goal([target])

    def schedule_next_goal(self):
        """결과 콜백에서 다음 목표를 '한 번만' 예약"""
        if not self._running:
            return

        self._cancel_next_goal_timer()

        delay = float(self.get_parameter('loop_delay_sec').value)
        # one-shot처럼 쓰기 위해 멤버로 저장하고 콜백에서 cancel
        self._next_goal_timer = self.create_timer(delay, self._oneshot_next_goal_cb)

    def _oneshot_next_goal_cb(self):
        # one-shot: 첫 실행 후 즉시 cancel
        self._cancel_next_goal_timer()
        self.send_next_goal_now()

    # --------------------
    # Action client
    # --------------------
    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self._action_client.wait_for_server()

        self._sending_goal = True
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._sending_goal = False
        self.goal_handle = future.result()
        self.goal_accept = self.goal_handle.accepted

        if not self.goal_accept:
            self.get_logger().error('Goal rejected.')
            # 재시도(다음 goal 예약)
            self.goal_handle = None
            self.goal_accept = False
            self.schedule_next_goal()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            res = future.result()
            status = res.status
            result = res.result
            missed_waypoints = result.missed_waypoints

            # goal 상태 정리(먼저!)
            self.goal_accept = False
            self.goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED:
                if not missed_waypoints:
                    self.get_logger().info('Arrived: all waypoints succeeded.')
                else:
                    self.get_logger().warn(f'Arrived but missed: {missed_waypoints}')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn('Goal canceled.')
            else:
                self.get_logger().warn(f'Goal finished with status={status}, missed={missed_waypoints}')

            # ===== "도착하면 다음 목표 생성" 로직 =====
            # 방금 목표가 ws2였다면 다음은 ws1, ws1이었다면 다음은 ws2
            self._next_target = 1 if self._next_target == 2 else 2

            # 다음 goal 예약
            self.schedule_next_goal()

        except Exception as e:
            self.get_logger().error(f'Exception in get_result_callback: {e}')
            # 상태 정리 후 재시도
            self.goal_accept = False
            self.goal_handle = None
            self.schedule_next_goal()

    # --------------------
    # Cancel / external cmd
    # --------------------
    def cancel_goal(self):
        if self.goal_handle is not None and self.goal_accept:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().warn('No goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal successfully cancelled.')
        else:
            self.get_logger().warn(f'Goal cancellation returned code: {cancel_response.return_code}')

        self.goal_accept = False
        self.goal_handle = None

        msg = String()
        msg.data = 'driving_done'
        self.control_from_amr.publish(msg)

    def control_from_center_callback(self, msg):
        if msg.data == 'start_loop':
            self.start_loop()
        elif msg.data in ['stop_loop', 'kill']:
            self.stop_loop()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()