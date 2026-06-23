import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sml_msgs.srv import ArmCommand
from arm_interfaces.srv import Cargo, GetTargetPose
from std_srvs.srv import Trigger
import rbpodo as rb
import numpy as np
import time
import threading


ROBOT_IP = "10.0.2.8"

HOME_JOINT_DEG   = np.array([-90.0,   0.0,  90.0, 0.0, 90.0, 0.0])
MOVING_JOINT_DEG = np.array([-90.0, -26.02, 140.8, 0.0, 65.22, 0.0])

# 슬롯별 웨이포인트 (joint, degree)
# 첫 번째 포인트는 HOME_JOINT_DEG와 동일하게 유지한다.
# 실제 이동에서는 정방향 첫 waypoint와 역방향 첫 waypoint를 스킵한다.
# load/unload가 동일한 단일 테이블을 공유한다 (slot 4도 load 값으로 통일).
SLOT_WAYPOINTS = {
    1: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-15.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([53.60, 23.71, 15.87, 3.85, 130.79, 0.0]),
        np.array([67.78, -2.06, 59.03, 4.11, 113.71, -20.45]),
    ],
    2: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-165.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-220.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-250.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-266.73, 9.62, 46.62, -1.82, 116.78, 2.62]),
    ],
    3: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-165.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-220.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-250.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-254.05, 12.14, 43.94, -3.51, 117.43, 14.43]),
    ],
    4: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-165.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-220.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-250.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-243.17, 18.56, 36.13, -4.91, 119.49, 24.47]),
    ],
    5: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-165.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-220.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-241.67, 1.06, 55.62, 0.0, 123.30, 28.34]),
    ],
    6: [
        np.array([-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]),
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-165.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-220.0, -11.96, 57.40, 0.0, 100.40, 0.0]),
        np.array([-261.20, -4.82, 61.32, 0.0, 123.49, 8.82]),
    ],
}

# 인덱스 0~5: 내려놓는 순서에 따라 사용 (unload 전용)
DELIVERY_WAYPOINTS = {
    0: [
        np.array([-106.29, 35.41, 98.92, 0.0, 45.67, -16.28]),
    ],
    1: [
        np.array([-91.40, 32.81, 103.23, 0.0, 43.95, -1.39]),
    ],
    2: [
        np.array([-75.10, 34.5, 100.44, 0.0, 45.06, 14.91]),
    ],
    3: [
        np.array([-78.43, 52.72, 68.95, 0.0, 58.33, 11.58]),
    ],
    4: [
        np.array([-90.71, 51.06, 71.88, 0.0, 57.06, -0.7]),
    ],
    5: [
        np.array([-103.28, 53.86, 66.91, 0.0, 59.23, -13.27]),
    ],
}

# --- LOAD 비전/오프셋 상수 ---
CAM_X_OFF = -51.0
CAM_Y_OFF = 32.0
LOAD_Z_DOWN_MM = 15.0
LOAD_Z_UP_MM = -15.0
Z_OFFSET = -85.0
Z_MARGIN = 40.0
SCAN_Y_OFFSETS_MM = [0.0, 100.0, -100.0, 180.0, -180.0]
SCAN_Y_AXIS_INDEX = 1
SCAN_SETTLE_TIME_SEC = 0.3
SCAN_VISION_RETRIES_PER_POSE = 1

# --- UNLOAD Z 상수 ---
UNLOAD_Z_DOWN_MM = 18.0
UNLOAD_Z_UP_MM = -18.0


J_VEL, J_ACC = 255, 255
L_VEL, L_ACC = 500, 800

MATERIAL_NAMES = {
    # --- Raw Materials ---
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
    # --- Products ---
    34: "battery",
    13: "magnet",
    81: "e_stop",
    442: "carrot",
    241: "traffic_light",
    462: "small_tree",
    711: "hammer",
    4482: "big_carrot",
    8518: "burger",
    48132: "ice_cream",
    46262: "big_tree",
}


class AmrRobotNode(Node):
    """load / unload 통합 오케스트레이터.

    /arm_command 서비스 하나로 LOAD / UNLOAD 를 모두 처리한다.
    request.action 이 'LOAD' 이면 적재 시퀀스, 'UNLOAD' 이면 출고 시퀀스를 돈다.
    로봇 연결·busy 락·서비스는 모두 단일 인스턴스로 공유한다.
    """

    def __init__(self):
        super().__init__('amr_robot_node')
        self.cbg = ReentrantCallbackGroup()

        self.robot = None
        self.rc = None
        self.robot_data = None
        self.robot_ready = False

        try:
            self.robot = rb.Cobot(ROBOT_IP)
            self.rc = rb.ResponseCollector()
            self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
            self.robot.set_speed_bar(self.rc, 1.0)
            self.robot_ready = True
            self.get_logger().info('[AMR] robot connected')
        except Exception as e:
            self.robot = None
            self.rc = None
            self.robot_ready = False
            self.get_logger().error(f'[AMR] robot connection error: {e}')

        # 현재 조인트 각도 읽기용 데이터 채널 (HOME 도착 여부 판정에 사용)
        try:
            self.robot_data = rb.CobotData(ROBOT_IP)
            self.get_logger().info('[AMR] data channel connected')
        except Exception as e:
            self.robot_data = None
            self.get_logger().warn(f'[AMR] data channel connect failed: {e}')

        self.vision_client = self.create_client(
            GetTargetPose, '/get_target_pose', callback_group=self.cbg)
        self.gripper_open_client = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cbg)
        self.gripper_grip_client = self.create_client(
            Trigger, '/gripper/grip', callback_group=self.cbg)
        self.cargo_client = self.create_client(
            Cargo, '/cargo', callback_group=self.cbg)
        self.srv = self.create_service(
            ArmCommand, '/amr_robot_command', self.arm_command_cb, callback_group=self.cbg)

        self._busy_lock = threading.Lock()
        self._busy = False
        self._at_home = False

        self.get_logger().info('[AMR] amr_robot_node started')

    # --- 상태 확인 헬퍼 ---

    def is_robot_ready(self):
        if not self.robot_ready or self.robot is None or self.rc is None:
            self.get_logger().error('[AMR] robot is not connected')
            return False
        return True

    def is_at_home(self, tol_deg=1.0):
        """현재 측정 조인트 각도(jnt_ang)를 읽어 HOME과 비교한다.
        데이터 채널이 없거나 읽기 실패 시 False를 반환해, 안전하게 실제 이동으로 폴백한다."""
        if self.robot_data is None:
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
            return bool(np.all(np.abs(cur - HOME_JOINT_DEG) <= tol_deg))
        except Exception as e:
            self.get_logger().warn(f'[AMR] is_at_home read failed: {e}')
            return False

    # --- 서비스 호출 헬퍼 ---

    def call_service(self, client, request, timeout=10.0):
        """Call a ROS2 service from inside callbacks without nested spinning.
        This node runs under MultiThreadedExecutor with a ReentrantCallbackGroup.
        The current callback thread waits on an Event, while another executor
        thread processes the service response.
        """
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'[AMR] service unavailable: {client.srv_name}')
                return None

            future = client.call_async(request)
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())

            if not done_event.wait(timeout=timeout):
                self.get_logger().error(f'[AMR] service timeout: {client.srv_name}')
                return None

            return future.result()
        except Exception as e:
            self.get_logger().error(f'[AMR] service call failed: {client.srv_name}: {e}')
            return None

    def call_vision(self, target_color, retries=3):
        for i in range(retries):
            req = GetTargetPose.Request()
            req.target_color = target_color
            req.target_size = ""
            res = self.call_service(self.vision_client, req, timeout=30)
            if res and res.success:
                return res
            self.get_logger().warn(f'[AMR] vision retry {i + 1}/{retries}')
            time.sleep(0.5)
        return None

    def _scan_y_delta(self, dy_mm):
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        delta[SCAN_Y_AXIS_INDEX] = float(dy_mm)
        return delta

    def return_scan_center(self, current_y_offset_mm):
        if abs(current_y_offset_mm) < 1e-6:
            return True
        return self.move_l_rel_checked(
            self._scan_y_delta(-current_y_offset_mm),
            label='scan return center',
        )

    def call_vision_with_y_scan(self, target_color):
        current_y_offset = 0.0

        for target_y_offset in SCAN_Y_OFFSETS_MM:
            delta_y = target_y_offset - current_y_offset
            if abs(delta_y) > 1e-6:
                if not self.move_l_rel_checked(
                    self._scan_y_delta(delta_y),
                    label=f'scan y offset {target_y_offset:.0f}mm',
                ):
                    self.get_logger().error(
                        f'[AMR] scan move failed: y={target_y_offset:.0f}mm')
                    self.return_scan_center(current_y_offset)
                    return None
                current_y_offset = target_y_offset

            time.sleep(SCAN_SETTLE_TIME_SEC)
            self.get_logger().info(
                f'[AMR] vision scan at y_offset={current_y_offset:.0f}mm')

            res = self.call_vision(
                target_color,
                retries=SCAN_VISION_RETRIES_PER_POSE,
            )
            if res:
                self.get_logger().info(
                    f'[AMR] vision success at y_offset={current_y_offset:.0f}mm')
                return res

        self.get_logger().warn('[AMR] vision scan failed at all y offsets')
        if not self.return_scan_center(current_y_offset):
            self.get_logger().error('[AMR] failed to return scan center')
        return None

    def call_gripper(self, grip: bool):
        client = self.gripper_grip_client if grip else self.gripper_open_client
        req = Trigger.Request()
        res = self.call_service(client, req, timeout=6.0)
        action_name = 'grip' if grip else 'open'
        if res and res.success:
            self.get_logger().info(f'[GRIPPER] {action_name}')
            return True
        self.get_logger().error(f'[GRIPPER] {action_name} failed')
        return False

    def call_cargo(self, action, slot=0, object_id=0):
        req = Cargo.Request()
        req.action = action
        req.slot = slot
        req.object_id = object_id
        return self.call_service(self.cargo_client, req)

    # --- 로봇 이동 헬퍼 ---

    def wait_move(self, timeout=10.0, label='move'):
        if not self.is_robot_ready():
            return False
        try:
            result = self.robot.wait_for_move_finished(self.rc, timeout=timeout)
            if result is False:
                self.get_logger().error(f'[AMR] {label} wait returned False')
                return False
            return True
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} wait failed: {e}')
            return False

    def move_j_checked(self, joints_deg, label='move_j', timeout=10.0):
        if not self.is_robot_ready():
            return False
        try:
            self.robot.move_j(self.rc, joints_deg, J_VEL, J_ACC)
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        return self.wait_move(timeout=timeout, label=label)

    def move_l_rel_checked(self, delta, label='move_l_rel', timeout=10.0):
        if not self.is_robot_ready():
            return False
        try:
            self.get_logger().info(f'[AMR] command start {label}: {delta}')
            self.robot.move_l_rel(
                self.rc,
                np.array(delta, dtype=float),
                L_VEL,
                L_ACC,
                rb.ReferenceFrame.Tool,
            )
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        ok = self.wait_move(timeout=timeout, label=label)
        if ok:
            self.get_logger().info(f'[AMR] command done {label}')
        return ok

    def go_home(self):
        # 이미 HOME에 있으면 제자리 move_j를 보내지 않는다.
        # (이동량 0인 move_j는 wait_for_move_finished가 완료 신호를 제대로 못 받아
        #  timeout까지 대기하면서 큰 지연을 유발할 수 있음)
        # _at_home   : 직전에 HOME 도달한 경우 빠른 스킵(데이터 채널 read 생략)
        # is_at_home(): 노드 시작 직후처럼 플래그가 없어도 실제 조인트가 HOME이면 스킵
        if self._at_home or self.is_at_home():
            self._at_home = True
            self.get_logger().info('[AMR] already at home, skip go_home')
            return True
        if self.move_j_checked(HOME_JOINT_DEG, label='go_home'):
            self._at_home = True
            return True
        return False

    def go_moving_pose(self):
        """HOME 복귀 후 AMR 주행용 이동 포즈로 이동한다.
        반드시 go_home() 이후에 호출할 것 (HOME이 안전한 경유점).
        이동 포즈에서는 _at_home 을 False 로 내린다."""
        if not self.move_j_checked(MOVING_JOINT_DEG, label='go_moving_pose'):
            return False
        self._at_home = False
        self.get_logger().info('[AMR] moving pose reached')
        return True

    # --- 웨이포인트 이동 (action별 테이블을 인자로 받음) ---

    def move_to_slot(self, slot):
        waypoints = SLOT_WAYPOINTS.get(slot)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        # 슬롯으로 이동하면 HOME을 벗어나므로 플래그를 내린다.
        # (이게 빠지면 이후 go_home()이 실제 위치와 무관하게 스킵될 수 있다.)
        self._at_home = False

        # 정방향 첫 번째 waypoint는 HOME_JOINT_DEG라서 스킵한다.
        move_waypoints = waypoints[1:]

        for idx, wp in enumerate(move_waypoints, start=2):
            if not self.move_j_checked(wp, label=f'move_to_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] slot={slot} reached')
        return True

    def return_from_slot(self, slot):
        waypoints = SLOT_WAYPOINTS.get(slot)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        # 역방향 첫 번째 waypoint는 방금 도착했던 슬롯 최종 자세라서 스킵한다.
        return_waypoints = list(reversed(waypoints))[1:]

        for idx, wp in enumerate(return_waypoints, start=2):
            if not self.move_j_checked(wp, label=f'return_from_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] returned from slot={slot}')
        self._at_home = True
        return True

    def move_to_delivery(self, delivery_idx):
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        # delivery로 이동하면 HOME을 벗어나므로 플래그를 내린다.
        # (return_from_slot에서 _at_home=True로 설정된 직후 호출되기 때문에 필수)
        self._at_home = False

        for idx, wp in enumerate(waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_delivery({delivery_idx}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] delivery position {delivery_idx} reached')
        return True

    def return_from_delivery(self, delivery_idx):
        # delivery 웨이포인트가 1개뿐이면, 역순 복귀 시 현재 자세로 제자리 move_j를
        # 보내게 되는데, 이동량 0인 move_j는 wait_for_move_finished가 완료 신호를
        # 제대로 못 받아 timeout까지 대기하면서 큰 지연을 유발한다.
        # delivery 직후에는 어차피 HOME으로 복귀하므로, 중간 경유 없이 바로 HOME으로 간다.
        # (waypoint가 여러 개로 늘어나면 마지막 자세를 제외한 경유점만 역순으로 탄다.)
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        if len(waypoints) > 1:
            return_waypoints = list(reversed(waypoints))[1:]
            for idx, wp in enumerate(return_waypoints, start=1):
                if not self.move_j_checked(wp, label=f'return_from_delivery({delivery_idx}) wp{idx}'):
                    return False

        self.get_logger().info(f'[AMR] returned from delivery position {delivery_idx}')
        return True

    # --- 서비스 콜백 (LOAD / UNLOAD 분기) ---

    def arm_command_cb(self, request, response):
        response.slots = []
        response.object_ids = []

        action = request.action.upper()
        if action not in ('LOAD', 'UNLOAD'):
            response.success = False
            response.message = f'unknown action: {request.action}'
            return response

        if not self.is_robot_ready():
            response.success = False
            response.message = 'robot not connected'
            return response

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response
            self._busy = True

        try:
            if action == 'LOAD':
                results = self.sequence_load_multi(list(request.object_ids))
            else:
                results = self.sequence_unload_multi(list(request.object_ids))

            success_all = bool(results) and all(r['success'] for r in results)
            response.success = success_all
            response.slots = [r['slot'] for r in results]
            response.object_ids = [r['object_id'] for r in results]
            response.message = ', '.join(r['message'] for r in results)
        except Exception as e:
            self.get_logger().error(f'[AMR] exception: {e}')
            response.success = False
            response.slots = []
            response.object_ids = []
            response.message = str(e)
        finally:
            with self._busy_lock:
                self._busy = False

        return response

    # --- LOAD 시퀀스 ---

    def sequence_load_multi(self, object_ids):
        results = []
        for object_id in object_ids:
            result = self.sequence_load(object_id)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] load failed at object_id={object_id}, stopping')
                break
        # 모든 적재(또는 중단) 후 HOME을 거쳐 이동 포즈로 전환한다.
        # (각 sequence_load 내부에서 이미 HOME 복귀가 완료되므로 go_home()은 플래그로 즉시 스킵됨)
        self.go_home()
        self.go_moving_pose()
        return results

    def sequence_load(self, object_id):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        target_color = MATERIAL_NAMES.get(object_id)
        if not target_color:
            self.get_logger().error(f'[AMR] unknown object_id: {object_id}')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'unknown object_id={object_id}',
            }

        vision_target = str(object_id)

        self.get_logger().info(f'[LOAD START] object_id={object_id}, target={target_color}')

        # 1. 빈 슬롯 확인
        res = self.call_cargo('FIND_EMPTY', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] no empty slot')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'no empty slot',
            }
        slot = res.slot
        self.get_logger().info(f'[CARGO] empty slot: {slot}')

        # 2. 초기화
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        if not self.go_home():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'go_home failed',
            }

        # 3. HOME 기준 center -> left -> right -> wide_left -> wide_right 순서로 측정
        p = self.call_vision_with_y_scan(vision_target)
        if not p:
            self.get_logger().error('[AMR] vision failed')
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'OBJECT_NOT_FOUND',
            }

        # 4. YAW + XY + Z접근 동시 이동
        #    move_l_rel(Tool)의 병진 성분은 이동 시작(HOME) 프레임 기준으로 적용되고,
        #    yaw(rz)는 tool Z축 방향 자체를 안 바꾸므로, HOME에서 측정한 dx/dy/z를
        #    한 모션에 합칠 수 있다. 단 물체 바로 위(Z_MARGIN)까지만 대각선으로 내려가고,
        #    최종 접근은 5번에서 수직으로 따로 한다. (대각선 최종접근은 파지 안정성 저하)
        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET
        #    NOTE: p.yaw 단위는 deg. 손목이 반대로 돌거나 단위가 rad이면
        #          rz 항(p.yaw)을 -p.yaw 또는 np.radians(p.yaw)로 조정할 것.
        self._at_home = False  # 이 이동부터 HOME을 벗어남
        if not self.move_l_rel_checked(
            [dy, dx, z_move - Z_MARGIN, 0.0, 0.0, p.yaw],
            label='yaw+xy+z approach',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'yaw+xy+z approach failed',
            }

        # 5. 수직 최종 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label='z final approach',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'z final approach failed',
            }
        time.sleep(0.5)

        # 6. 그리퍼 grip
        if not self.call_gripper(True):
            self.get_logger().error('[AMR] grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                label='retreat after grip failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'grip failed',
            }

        # 7. Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label='lift after grip',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'lift after grip failed',
            }

        # 8. 웨이포인트 순서대로 슬롯으로 이동
        if not self.move_to_slot(slot):
            self.get_logger().error('[AMR] move to slot failed')
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 9. Z 하강 -> open -> Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
            label='place z down',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z down failed',
            }

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                [0.0, 0.0, LOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                label='retreat after open failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'final gripper open failed',
            }

        if not self.move_l_rel_checked(
            [0.0, 0.0, LOAD_Z_UP_MM, 0.0, 0.0, 0.0],
            label='place z up',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z up failed',
            }

        # 10. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_slot(slot):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'return from slot failed',
            }

        # 11. 카고 기록
        res = self.call_cargo('SET', slot=slot, object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo SET failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'loaded physically but cargo SET failed',
            }

        self.get_logger().info(f'[LOAD DONE] object_id={object_id}, slot={slot}')
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'load success',
        }

    # --- UNLOAD 시퀀스 ---

    def sequence_unload_multi(self, object_ids):
        results = []
        for idx, object_id in enumerate(object_ids):
            result = self.sequence_unload(object_id, idx)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] unload failed at object_id={object_id}, stopping')
                break
        # 모든 물체 처리(또는 중단) 후 HOME을 거쳐 이동 포즈로 전환한다.
        # 직전 물체에서 이미 HOME에 와 있으면 _at_home 플래그로 즉시 스킵된다.
        self.go_home()
        self.go_moving_pose()
        return results

    def sequence_unload(self, object_id, delivery_idx):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        self.get_logger().info(f'[UNLOAD START] object_id={object_id}, delivery_idx={delivery_idx}')

        # 1. 슬롯 확인
        res = self.call_cargo('FIND_OBJECT', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error(f'[AMR] object_id={object_id} not found in cargo')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'object not found: {object_id}',
            }
        slot = res.slot
        self.get_logger().info(f'[CARGO] object found: slot={slot}')

        # 2. 초기화
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        if not self.go_home():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'go_home failed',
            }

        # 3. 웨이포인트 순서대로 슬롯으로 이동
        if not self.move_to_slot(slot):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 4. Z 하강
        self.get_logger().info('[AMR] start slot z down')
        if not self.move_l_rel_checked(
            [0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
            label='slot z down',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot z down failed',
            }

        # 5. 그리퍼 grip
        if not self.call_gripper(True):
            self.get_logger().error('[AMR] grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                label='retreat after grip failure',
            )
            self.return_from_slot(slot)
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'grip failed',
            }

        # 6. Z 상승
        self.get_logger().info('[AMR] start slot z up')
        if not self.move_l_rel_checked(
            [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
            label='slot z up',
        ):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot z up failed',
            }

        # 7. 슬롯에서 물체를 들어 올렸으므로 cargo 상태를 먼저 비운다.
        # 이후 복귀 실패가 나도 cargo_manager의 슬롯 상태는 실제 물리 상태와 맞는다.
        res = self.call_cargo('CLEAR', slot=slot)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo CLEAR failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'object picked physically but cargo CLEAR failed',
            }

        # 8. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_slot(slot):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'cargo CLEAR done, but return from slot failed',
            }

        # 9. 배달 위치로 이동
        if not self.move_to_delivery(delivery_idx):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'move to delivery failed',
            }

        # 10. Z 하강 -> open -> Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
            label='delivery z down',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'delivery z down failed',
            }

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                label='retreat after delivery open failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'final gripper open failed',
            }

        if not self.move_l_rel_checked(
            [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
            label='delivery z up',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'delivery z up failed',
            }

        # 11. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_delivery(delivery_idx):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'return from delivery failed',
            }

        # 12. delivery 자세에서 곧장 다음 물체로 가면 큰 단일 관절 이동이 생겨
        #     느리므로, 물체 1개 처리가 끝날 때마다 HOME으로 복귀해 둔다.
        #     (다음 sequence_unload의 go_home()은 _at_home 플래그로 즉시 스킵된다.)
        if not self.go_home():
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'go_home after delivery failed',
            }

        self.get_logger().info(
            f'[UNLOAD DONE] object_id={object_id}, slot={slot}, delivery_idx={delivery_idx}'
        )
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'unload success',
        }


def main(args=None):
    rclpy.init(args=args)
    node = AmrRobotNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
