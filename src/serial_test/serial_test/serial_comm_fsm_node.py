# file: serial_test/serial_comm_fsm_node.py
#
# Serial FSM (Dual Role)
# - role=autonomy    : Robot A side (Nav waypoint ARRIVED:<wp> -> comm.md serial ARRIVED,WSx,TASK -> wait DONE -> publish DONE:<wp>)
# - role=manipulator : Robot B side (serial ARRIVED,WSx,TASK -> publish manip_task_cmd -> wait manip_task_result -> serial DONE,WSx,TASK)
#
# comm.md on SERIAL line is strictly CSV:
#   A -> B: ARRIVED,WSx,PICK3 | ARRIVED,WSx,PLACE3
#   B -> A: DONE,WSx,PICK3    | DONE,WSx,PLACE3
#   (optional) FAIL,WSx,TASK,REASON
#
# Note:
# - yaml_waypoint_node uses "ARRIVED:<wp_name>" and "DONE:<wp_name>" internally.
#   This node translates between internal colon flags and comm.md CSV over serial.

from __future__ import annotations

import threading
import time
import re
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


# -------------------------
# Parsing helpers
# -------------------------
@dataclass
class CommMsg:
    msg_type: str
    ws: Optional[str] = None
    task: Optional[str] = None
    reason: Optional[str] = None


def _split_csv(text: str) -> List[str]:
    return [p.strip() for p in (text or "").split(",") if p.strip()]


def parse_comm_csv(text: str) -> Optional[CommMsg]:
    s = (text or "").strip()
    if not s:
        return None
    parts = _split_csv(s)
    if not parts:
        return None
    msg_type = parts[0].strip()
    ws = parts[1].strip() if len(parts) >= 2 else None
    task = parts[2].strip() if len(parts) >= 3 else None
    reason = parts[3].strip() if len(parts) >= 4 else None
    return CommMsg(msg_type=msg_type, ws=ws, task=task, reason=reason)


def format_comm_csv(msg_type: str, ws: str, task: str, reason: Optional[str] = None) -> str:
    if reason:
        return f"{msg_type},{ws},{task},{reason}"
    return f"{msg_type},{ws},{task}"


def parse_colon_flag(text: str, prefix: str) -> Optional[str]:
    # expected: "PREFIX:payload"
    s = (text or "").strip()
    if not s:
        return None
    pfx = f"{prefix}:"
    if not s.startswith(pfx):
        return None
    payload = s[len(pfx):].strip()
    return payload if payload else None


def is_valid_ws(ws: str, ws_prefix: str = "WS") -> bool:
    return bool(ws) and re.match(rf"^{re.escape(ws_prefix)}\d+$", ws.strip()) is not None


# -------------------------
# Serial line I/O
# -------------------------
class SerialLineIO:
    def __init__(
        self,
        logger,
        port: str,
        baudrate: int,
        timeout: float,
        line_ending: str,
        auto_reconnect: bool,
        reconnect_sec: float,
    ):
        self._logger = logger
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._line_ending = line_ending
        self._auto_reconnect = auto_reconnect
        self._reconnect_sec = reconnect_sec

        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._last_try = 0.0

    def ensure(self) -> bool:
        if self._ser is not None and self._ser.is_open:
            return True

        now = time.time()
        if not self._auto_reconnect and self._ser is not None:
            return False
        if now - self._last_try < self._reconnect_sec:
            return False

        self._last_try = now
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=self._timeout)
            self._logger.info(f"[SERIAL] connected: {self._port} @ {self._baudrate}")
            return True
        except Exception as exc:
            self._ser = None
            self._logger.warn(f"[SERIAL] open failed ({self._port}): {exc}")
            return False

    def close(self):
        with self._lock:
            try:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None

    def write_line(self, text: str) -> bool:
        if not self.ensure():
            return False
        data = (text or "")
        if self._line_ending and not data.endswith(self._line_ending):
            data = f"{data}{self._line_ending}"
        try:
            with self._lock:
                self._ser.write(data.encode("utf-8"))
            return True
        except Exception as exc:
            self._logger.warn(f"[SERIAL] write failed: {exc}")
            self.close()
            return False

    def read_line(self) -> Optional[str]:
        if not self.ensure():
            return None
        try:
            with self._lock:
                line = self._ser.readline()
            if not line:
                return None
            text = line.decode("utf-8", errors="ignore").strip("\r\n")
            return text if text else None
        except Exception as exc:
            self._logger.warn(f"[SERIAL] read failed: {exc}")
            self.close()
            return None


# -------------------------
# FSM states
# -------------------------
class AState(str, Enum):
    IDLE = "IDLE"
    WAIT_DONE = "WAIT_DONE"


class BState(str, Enum):
    IDLE = "IDLE"
    WAIT_MANIP_RESULT = "WAIT_MANIP_RESULT"


# -------------------------
# Main node
# -------------------------
class SerialCommFSM(Node):
    def __init__(self):
        super().__init__("serial_comm_fsm_node")

        # role
        self.declare_parameter("role", "autonomy")  # autonomy | manipulator

        # serial
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 0.1)
        self.declare_parameter("line_ending", "\n")
        self.declare_parameter("auto_reconnect", True)
        self.declare_parameter("reconnect_sec", 1.0)

        # comm.md keywords (serial CSV)
        self.declare_parameter("arrived_type", "ARRIVED")
        self.declare_parameter("done_type", "DONE")
        self.declare_parameter("fail_type", "FAIL")
        self.declare_parameter("ws_prefix", "WS")
        self.declare_parameter("validate_ws", True)
        self.declare_parameter("allowed_tasks", ["PICK3", "PLACE3"])
        self.declare_parameter("enable_fail_tx", True)

        # Robot A: nav handshake topics (colon style)
        self.declare_parameter("wp_arrive_topic", "wp_arrived")
        self.declare_parameter("wp_done_topic", "wp_done")
        self.declare_parameter("wp_arrive_prefix", "ARRIVED")
        self.declare_parameter("wp_done_prefix", "DONE")

        # Robot A: mapping & plan
        self.declare_parameter("ws_map_yaml", "{work_station1: WS1, work_station2: WS2}")
        self.declare_parameter("task_plan_yaml", "{WS1: [PICK3], WS2: [PLACE3, PICK3]}")

        # Robot A: retry/timeout
        self.declare_parameter("done_timeout_sec", 120.0)
        self.declare_parameter("resend_sec", 2.0)
        self.declare_parameter("max_resends", 5)
        self.declare_parameter("fail_policy", "stop")  # stop | skip_waypoint

        # Robot B: manipulator topics
        self.declare_parameter("manip_cmd_topic", "manip_task_cmd")
        self.declare_parameter("manip_result_topic", "manip_task_result")
        self.declare_parameter("busy_policy", "ignore")  # ignore | reject
        self.declare_parameter("dedup_done_sec", 10.0)

        # logging
        self.declare_parameter("log_serial_rx", True)
        self.declare_parameter("log_serial_tx", True)
        self.declare_parameter("log_state", True)

        # load params
        self._role = str(self.get_parameter("role").value).strip().lower()

        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("timeout").value)
        line_ending = str(self.get_parameter("line_ending").value)
        auto_reconnect = bool(self.get_parameter("auto_reconnect").value)
        reconnect_sec = float(self.get_parameter("reconnect_sec").value)

        self._arrived_type = str(self.get_parameter("arrived_type").value)
        self._done_type = str(self.get_parameter("done_type").value)
        self._fail_type = str(self.get_parameter("fail_type").value)
        self._ws_prefix = str(self.get_parameter("ws_prefix").value)
        self._validate_ws = bool(self.get_parameter("validate_ws").value)
        self._allowed_tasks = list(self.get_parameter("allowed_tasks").value)
        self._enable_fail_tx = bool(self.get_parameter("enable_fail_tx").value)

        self._log_serial_rx = bool(self.get_parameter("log_serial_rx").value)
        self._log_serial_tx = bool(self.get_parameter("log_serial_tx").value)
        self._log_state = bool(self.get_parameter("log_state").value)

        self._serial = SerialLineIO(
            logger=self.get_logger(),
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            line_ending=line_ending,
            auto_reconnect=auto_reconnect,
            reconnect_sec=reconnect_sec,
        )

        # shared RX poll timer
        self._rx_timer = self.create_timer(0.02, self._poll_serial)

        # role-specific init
        if self._role == "autonomy":
            self._init_autonomy()
        elif self._role == "manipulator":
            self._init_manipulator()
        else:
            raise RuntimeError(f"Unknown role: {self._role} (use 'autonomy' or 'manipulator')")

        self.get_logger().info(f"SerialCommFSM up | role={self._role} port={port} baud={baudrate}")

    # -------------------------
    # Robot A (autonomy) logic
    # -------------------------
    def _init_autonomy(self):
        self._wp_arrive_topic = str(self.get_parameter("wp_arrive_topic").value)
        self._wp_done_topic = str(self.get_parameter("wp_done_topic").value)
        self._wp_arrive_prefix = str(self.get_parameter("wp_arrive_prefix").value)
        self._wp_done_prefix = str(self.get_parameter("wp_done_prefix").value)

        self._ws_map = self._parse_yaml_dict_str(self.get_parameter("ws_map_yaml").value)
        self._task_plan = self._parse_yaml_dict_list_str(self.get_parameter("task_plan_yaml").value)

        self._done_timeout_sec = float(self.get_parameter("done_timeout_sec").value)
        self._resend_sec = float(self.get_parameter("resend_sec").value)
        self._max_resends = int(self.get_parameter("max_resends").value)
        self._fail_policy = str(self.get_parameter("fail_policy").value).strip().lower()

        # subscribe arrival from nav node
        self._wp_arrive_sub = self.create_subscription(String, self._wp_arrive_topic, self._on_wp_arrive, 10)
        # publish done back to nav node
        self._wp_done_pub = self.create_publisher(String, self._wp_done_topic, 10)

        # A FSM runtime
        self._a_state: AState = AState.IDLE
        self._current_wp: Optional[str] = None
        self._current_ws: Optional[str] = None
        self._tasks: List[str] = []
        self._task_idx: int = 0

        self._expected: Optional[Tuple[str, str]] = None  # (ws, task)
        self._sent_time: float = 0.0
        self._last_resend: float = 0.0
        self._resend_count: int = 0

        self._tick = self.create_timer(0.1, self._tick_autonomy)

    def _on_wp_arrive(self, msg: String):
        raw = (msg.data or "").strip()
        wp = parse_colon_flag(raw, self._wp_arrive_prefix)
        if wp is None:
            return

        # already handling a station -> ignore duplicate nav-arrive spam
        if self._a_state != AState.IDLE:
            return

        ws = str(self._ws_map.get(wp, "")).strip()
        if not ws:
            self.get_logger().warn(f"[A] No WS mapping for waypoint '{wp}'. Add ws_map_yaml.")
            return

        if self._validate_ws and not is_valid_ws(ws, self._ws_prefix):
            self.get_logger().warn(f"[A] Mapped WS '{ws}' invalid (expected {self._ws_prefix}N).")
            return

        tasks = self._task_plan.get(ws, [])
        if not tasks:
            self.get_logger().warn(f"[A] No task plan for {ws}. Add task_plan_yaml.")
            return

        # validate tasks
        for t in tasks:
            if self._allowed_tasks and t not in self._allowed_tasks:
                self.get_logger().warn(f"[A] Task '{t}' not allowed. allowed_tasks={self._allowed_tasks}")
                return

        self._current_wp = wp
        self._current_ws = ws
        self._tasks = tasks
        self._task_idx = 0

        self.get_logger().info(f"[A] Arrived wp={wp} -> ws={ws} tasks={tasks}")
        self._send_arrived_current_task()

    def _send_arrived_current_task(self):
        if self._current_ws is None or self._current_wp is None:
            return
        if self._task_idx >= len(self._tasks):
            # station tasks complete -> unblock nav node
            done_flag = String()
            done_flag.data = f"{self._wp_done_prefix}:{self._current_wp}"
            self._wp_done_pub.publish(done_flag)
            self.get_logger().info(f"[A->NAV] {done_flag.data} (station complete)")

            # reset
            self._a_state = AState.IDLE
            self._current_wp = None
            self._current_ws = None
            self._tasks = []
            self._task_idx = 0
            self._expected = None
            return

        ws = self._current_ws
        task = self._tasks[self._task_idx]
        line = format_comm_csv(self._arrived_type, ws, task)

        ok = self._serial.write_line(line)
        if ok and self._log_serial_tx:
            self.get_logger().info(f"[A->SERIAL] {line}")

        now = time.time()
        self._expected = (ws, task)
        self._sent_time = now
        self._last_resend = now
        self._resend_count = 0
        self._a_state = AState.WAIT_DONE
        self._log_state_autonomy()

    def _tick_autonomy(self):
        if self._a_state != AState.WAIT_DONE or self._expected is None:
            return

        now = time.time()
        ws, task = self._expected

        # timeout
        if self._done_timeout_sec > 0.0 and (now - self._sent_time) > self._done_timeout_sec:
            self.get_logger().warn(f"[A] DONE timeout for {ws},{task} (> {self._done_timeout_sec:.1f}s)")
            if self._fail_policy == "skip_waypoint":
                # unblock nav anyway
                if self._current_wp is not None:
                    done_flag = String()
                    done_flag.data = f"{self._wp_done_prefix}:{self._current_wp}"
                    self._wp_done_pub.publish(done_flag)
                    self.get_logger().warn(f"[A->NAV] {done_flag.data} (timeout skip)")
                self._a_state = AState.IDLE
            else:
                # stop: keep nav waiting
                self._a_state = AState.IDLE
            self._expected = None
            return

        # resend
        if self._resend_sec > 0.0 and (now - self._last_resend) >= self._resend_sec:
            if self._resend_count >= self._max_resends:
                self.get_logger().warn(f"[A] Max resends reached for {ws},{task}")
                return
            line = format_comm_csv(self._arrived_type, ws, task)
            ok = self._serial.write_line(line)
            self._resend_count += 1
            self._last_resend = now
            if ok and self._log_serial_tx:
                self.get_logger().warn(f"[A->SERIAL] RESEND {line} ({self._resend_count}/{self._max_resends})")

    def _handle_serial_line_autonomy(self, line: str):
        msg = parse_comm_csv(line)
        if msg is None:
            return
        if msg.msg_type != self._done_type and msg.msg_type != self._fail_type:
            return

        if self._a_state != AState.WAIT_DONE or self._expected is None:
            return

        ws = (msg.ws or "").strip()
        task = (msg.task or "").strip()
        if not ws or not task:
            return

        exp_ws, exp_task = self._expected
        if ws != exp_ws or task != exp_task:
            return

        # expected response
        if msg.msg_type == self._done_type:
            if self._log_serial_rx:
                self.get_logger().info(f"[SERIAL->A] {line} (matched)")
            self._expected = None
            self._task_idx += 1
            self._send_arrived_current_task()
            return

        if msg.msg_type == self._fail_type:
            reason = (msg.reason or "").strip() or "FAIL"
            self.get_logger().warn(f"[A] FAIL received for {ws},{task}: {reason}")
            if self._fail_policy == "skip_waypoint":
                self._expected = None
                self._task_idx = len(self._tasks)  # mark station complete
                self._send_arrived_current_task()
            else:
                self._expected = None
                self._a_state = AState.IDLE

    def _log_state_autonomy(self):
        if not self._log_state:
            return
        self.get_logger().info(f"[A-FSM] state={self._a_state} wp={self._current_wp} ws={self._current_ws} idx={self._task_idx}")

    # -------------------------
    # Robot B (manipulator) logic
    # -------------------------
    def _init_manipulator(self):
        self._manip_cmd_topic = str(self.get_parameter("manip_cmd_topic").value)
        self._manip_result_topic = str(self.get_parameter("manip_result_topic").value)
        self._busy_policy = str(self.get_parameter("busy_policy").value).strip().lower()
        self._dedup_done_sec = float(self.get_parameter("dedup_done_sec").value)

        self._manip_cmd_pub = self.create_publisher(String, self._manip_cmd_topic, 10)
        self._manip_result_sub = self.create_subscription(String, self._manip_result_topic, self._on_manip_result, 10)

        self._b_state: BState = BState.IDLE
        self._pending_ws: Optional[str] = None
        self._pending_task: Optional[str] = None
        self._pending_since: float = 0.0

        self._last_done_key: Optional[Tuple[str, str]] = None
        self._last_done_time: float = 0.0

        self._log_state_manipulator()

    def _handle_serial_line_manipulator(self, line: str):
        msg = parse_comm_csv(line)
        if msg is None:
            return
        if msg.msg_type != self._arrived_type:
            return

        ws = (msg.ws or "").strip()
        task = (msg.task or "").strip()
        if not ws or not task:
            return

        if self._validate_ws and not is_valid_ws(ws, self._ws_prefix):
            self.get_logger().warn(f"[B] invalid ws: {ws}")
            return

        if self._allowed_tasks and task not in self._allowed_tasks:
            self.get_logger().warn(f"[B] unknown task: {task}")
            if self._enable_fail_tx:
                self._send_fail(ws, task, "UNKNOWN_TASK")
            return

        # dedup: resend DONE if duplicate arrives shortly after completion
        if self._last_done_key == (ws, task) and (time.time() - self._last_done_time) <= self._dedup_done_sec:
            self.get_logger().warn(f"[B] duplicate ARRIVED; re-ACK DONE only: {ws},{task}")
            self._send_done(ws, task)
            return

        # busy handling
        if self._b_state != BState.IDLE:
            if self._pending_ws == ws and self._pending_task == task:
                return
            if self._busy_policy == "reject":
                self.get_logger().warn(f"[B] busy reject: {ws},{task}")
                if self._enable_fail_tx:
                    self._send_fail(ws, task, "BUSY")
            return

        # accept -> trigger manip task
        self._pending_ws = ws
        self._pending_task = task
        self._pending_since = time.time()
        self._b_state = BState.WAIT_MANIP_RESULT
        self._log_state_manipulator()

        cmd = String()
        cmd.data = task  # WS ignored for manip
        self._manip_cmd_pub.publish(cmd)
        self.get_logger().info(f"[B->MANIP_CMD] {cmd.data} (ws_saved={ws})")

    def _on_manip_result(self, msg: String):
        if self._b_state != BState.WAIT_MANIP_RESULT or self._pending_ws is None or self._pending_task is None:
            return

        raw = (msg.data or "").strip()
        if not raw:
            return

        ws = self._pending_ws
        task = self._pending_task

        parts = _split_csv(raw)
        head = parts[0].strip().upper() if parts else ""

        if head == "DONE":
            self._send_done(ws, task)
            self._mark_done(ws, task)
            self._reset_pending_b()
            return

        if head == "FAIL":
            reason = parts[1].strip() if len(parts) >= 2 else "FAIL"
            if self._enable_fail_tx:
                self._send_fail(ws, task, reason)
            self._reset_pending_b()
            return

        # unknown -> treat as fail
        if self._enable_fail_tx:
            self._send_fail(ws, task, "BAD_RESULT")
        self._reset_pending_b()

    def _send_done(self, ws: str, task: str):
        line = format_comm_csv(self._done_type, ws, task)
        ok = self._serial.write_line(line)
        if ok and self._log_serial_tx:
            self.get_logger().info(f"[B->SERIAL] {line}")

    def _send_fail(self, ws: str, task: str, reason: str):
        line = format_comm_csv(self._fail_type, ws, task, reason)
        ok = self._serial.write_line(line)
        if ok and self._log_serial_tx:
            self.get_logger().info(f"[B->SERIAL] {line}")

    def _mark_done(self, ws: str, task: str):
        self._last_done_key = (ws, task)
        self._last_done_time = time.time()

    def _reset_pending_b(self):
        self._pending_ws = None
        self._pending_task = None
        self._pending_since = 0.0
        self._b_state = BState.IDLE
        self._log_state_manipulator()

    def _log_state_manipulator(self):
        if not self._log_state:
            return
        if self._role != "manipulator":
            return
        self.get_logger().info(f"[B-FSM] state={self._b_state} pending=({self._pending_ws},{self._pending_task})")

    # -------------------------
    # Serial polling (shared)
    # -------------------------
    def _poll_serial(self):
        line = self._serial.read_line()
        if not line:
            return

        if self._log_serial_rx:
            self.get_logger().info(f"[SERIAL_RX] {line}")

        if self._role == "autonomy":
            self._handle_serial_line_autonomy(line)
        else:
            self._handle_serial_line_manipulator(line)

    # -------------------------
    # YAML parsing utilities (string -> dict)
    # -------------------------
    def _parse_yaml_dict_str(self, text: str) -> Dict[str, str]:
        try:
            import yaml
            d = yaml.safe_load(str(text)) or {}
            if isinstance(d, dict):
                return {str(k): str(v) for k, v in d.items()}
        except Exception as exc:
            self.get_logger().warn(f"ws_map_yaml parse failed: {exc}")
        return {}

    def _parse_yaml_dict_list_str(self, text: str) -> Dict[str, List[str]]:
        try:
            import yaml
            d = yaml.safe_load(str(text)) or {}
            out: Dict[str, List[str]] = {}
            if isinstance(d, dict):
                for k, v in d.items():
                    if isinstance(v, list):
                        out[str(k)] = [str(x) for x in v]
                    else:
                        out[str(k)] = [str(v)]
                return out
        except Exception as exc:
            self.get_logger().warn(f"task_plan_yaml parse failed: {exc}")
        return {}

    def destroy_node(self):
        self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
