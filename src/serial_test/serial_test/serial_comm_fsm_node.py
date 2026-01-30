# file: serial_test/serial_comm_fsm_node.py
#
# Serial 통신(FSM) 노드 (Robot B: Manipulator Side 기준)
# - 입력:  /serial_rx (std_msgs/String)  -> comm.md 포맷의 플래그 수신
# - 출력:  /serial_tx (std_msgs/String)  -> comm.md 포맷의 DONE(옵션: FAIL) 송신
# - 로컬 동작 트리거:
#     /manip_task_cmd    (std_msgs/String)  예) "WS1,PICK3"
#   로컬 완료/실패 수신:
#     /manip_task_result (std_msgs/String)  예) "DONE" 또는 "FAIL,NO_OBJECT"
#
# comm.md 준수(기본):
# - 수신 플래그:  "ARRIVED,WSx,PICK3"  또는 "ARRIVED,WSx,PLACE3"
# - 송신 플래그:  "DONE,WSx,PICK3"     또는 "DONE,WSx,PLACE3"
# - (옵션) 실패 송신: "FAIL,WSx,PICK3,REASON"
#
# 주의:
# - FAIL은 comm.md 본문에 언급되지만 TYPE 정의에는 빠져있어서, 기본값은 "FAIL 송신 비활성화"로 둠.
# - 필요하면 enable_fail_tx=True로 켜면 됨.

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# -------------------------
# FSM State
# -------------------------
class FSMState(str, Enum):
    IDLE = "IDLE"                # 대기
    WAIT_MANIP_RESULT = "WAIT"   # 매니퓰레이터 결과 대기


# -------------------------
# Parsed comm.md message
# -------------------------
@dataclass
class CommMsg:
    msg_type: str                 # ARRIVED / DONE / FAIL ...
    ws: Optional[str] = None      # WS1, WS2 ...
    task: Optional[str] = None    # PICK3, PLACE3
    reason: Optional[str] = None  # 실패 원인(옵션)


# -------------------------
# Helpers
# -------------------------
def _split_csv(text: str) -> List[str]:
    return [p.strip() for p in text.split(",") if p.strip()]


def parse_comm_md(text: str, allow_legacy: bool = True) -> Optional[CommMsg]:
    """
    comm.md 기본 포맷(권장):
      TYPE,WS,TASK
      (옵션) FAIL,WS,TASK,REASON

    레거시(호환) 포맷(옵션):
      TYPE:NAME
      TYPE:WS:TASK
      DONE (단독)
      FAIL:REASON (단독)
    """
    s = (text or "").strip()
    if not s:
        return None

    # 1) CSV-like (comm.md)
    parts = _split_csv(s)
    if len(parts) >= 3:
        msg_type, ws, task = parts[0], parts[1], parts[2]
        reason = parts[3] if len(parts) >= 4 else None
        return CommMsg(msg_type=msg_type, ws=ws, task=task, reason=reason)

    # 2) Legacy (optional)
    if allow_legacy and ":" in s:
        seg = [p.strip() for p in s.split(":") if p.strip()]
        if len(seg) == 1:
            return CommMsg(msg_type=seg[0])
        if len(seg) == 2:
            # e.g., ARRIVED:WS1_PICK3  or DONE:WS1
            return CommMsg(msg_type=seg[0], ws=seg[1])
        if len(seg) >= 3:
            # e.g., ARRIVED:WS1:PICK3  or FAIL:WS1:PICK3:REASON
            msg_type = seg[0]
            ws = seg[1]
            task = seg[2]
            reason = seg[3] if len(seg) >= 4 else None
            return CommMsg(msg_type=msg_type, ws=ws, task=task, reason=reason)

    # 3) Single token (optional)
    if allow_legacy:
        up = s.upper()
        if up == "DONE":
            return CommMsg(msg_type="DONE")
        if up.startswith("FAIL"):
            # "FAIL" or "FAIL,REASON"(CSV already handled) or "FAIL:REASON"(legacy handled)
            return CommMsg(msg_type="FAIL")

    return None


def format_comm_csv(msg_type: str, ws: str, task: str, reason: Optional[str] = None) -> str:
    # comm.md 준수: TYPE,WS,TASK (+ optional reason)
    if reason:
        return f"{msg_type},{ws},{task},{reason}"
    return f"{msg_type},{ws},{task}"


# -------------------------
# Node
# -------------------------
class SerialCommFSM(Node):
    """
    Robot B(Manipulator side) FSM:
      - serial_rx에서 ARRIVED,WS,TASK 수신
      - 로컬 매니퓰레이터 동작 트리거 토픽 publish (WS,TASK)
      - 로컬 결과 수신 후 serial_tx로 DONE,WS,TASK 송신
      - 의도하지 않은 명령/형식은 예외 처리(로그 + 무시 또는 옵션 FAIL 송신)
    """

    def __init__(self):
        super().__init__("serial_comm_fsm")

        # ----- Parameters -----
        self.declare_parameter("serial_rx_topic", "serial_rx")
        self.declare_parameter("serial_tx_topic", "serial_tx")

        self.declare_parameter("arrived_type", "ARRIVED")
        self.declare_parameter("done_type", "DONE")
        self.declare_parameter("fail_type", "FAIL")

        # comm.md 준수 기본: CSV. (레거시 허용은 수신만)
        self.declare_parameter("allow_legacy_rx_format", True)
        self.declare_parameter("enable_fail_tx", False)  # 기본 OFF (comm.md 정의 불일치 가능성)

        # 동작 트리거/결과 토픽 (로컬)
        self.declare_parameter("manip_cmd_topic", "manip_task_cmd")
        self.declare_parameter("manip_result_topic", "manip_task_result")

        # 허용 목록(비어 있으면 모두 허용)
        self.declare_parameter("allowed_ws", [])          # 예: ["WS1", "WS2"]
        self.declare_parameter("allowed_tasks", ["PICK3", "PLACE3"])

        # Busy/Timeout 정책
        self.declare_parameter("busy_policy", "ignore")   # ignore | reject
        self.declare_parameter("timeout_sec", 120.0)

        # 로깅
        self.declare_parameter("log_raw_rx", True)
        self.declare_parameter("log_state", True)

        # ----- Load params -----
        self._serial_rx_topic = self.get_parameter("serial_rx_topic").value
        self._serial_tx_topic = self.get_parameter("serial_tx_topic").value

        self._arrived_type = self.get_parameter("arrived_type").value
        self._done_type = self.get_parameter("done_type").value
        self._fail_type = self.get_parameter("fail_type").value

        self._allow_legacy = bool(self.get_parameter("allow_legacy_rx_format").value)
        self._enable_fail_tx = bool(self.get_parameter("enable_fail_tx").value)

        self._manip_cmd_topic = self.get_parameter("manip_cmd_topic").value
        self._manip_result_topic = self.get_parameter("manip_result_topic").value

        self._allowed_ws = list(self.get_parameter("allowed_ws").value)
        self._allowed_tasks = list(self.get_parameter("allowed_tasks").value)

        self._busy_policy = str(self.get_parameter("busy_policy").value).strip().lower()
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)

        self._log_raw_rx = bool(self.get_parameter("log_raw_rx").value)
        self._log_state = bool(self.get_parameter("log_state").value)

        # ----- Publishers/Subscribers -----
        self._serial_tx_pub = self.create_publisher(String, self._serial_tx_topic, 10)
        self._serial_rx_sub = self.create_subscription(String, self._serial_rx_topic, self._on_serial_rx, 10)

        self._manip_cmd_pub = self.create_publisher(String, self._manip_cmd_topic, 10)
        self._manip_result_sub = self.create_subscription(String, self._manip_result_topic, self._on_manip_result, 10)

        # ----- FSM state -----
        self._state: FSMState = FSMState.IDLE
        self._pending_ws: Optional[str] = None
        self._pending_task: Optional[str] = None
        self._pending_since: float = 0.0

        # Timeout watchdog
        self._tick = self.create_timer(0.1, self._on_tick)

        self.get_logger().info(
            f"SerialCommFSM started | rx={self._serial_rx_topic} tx={self._serial_tx_topic} "
            f"| manip_cmd={self._manip_cmd_topic} manip_result={self._manip_result_topic}"
        )

    # -------------------------
    # Core callbacks
    # -------------------------
    def _on_serial_rx(self, msg: String):
        raw = (msg.data or "").strip()
        if self._log_raw_rx:
            self.get_logger().info(f"[SERIAL_RX] {raw}")

        parsed = parse_comm_md(raw, allow_legacy=self._allow_legacy)
        if parsed is None:
            self._handle_unexpected(f"invalid_format:{raw}")
            return

        # Normalize type
        msg_type = (parsed.msg_type or "").strip()
        if not msg_type:
            self._handle_unexpected(f"empty_type:{raw}")
            return

        # Only handle ARRIVED on manip side
        if msg_type != self._arrived_type:
            # DONE/FAIL 등이 들어오면 로깅만 하고 무시(역할 분리)
            self.get_logger().debug(f"Ignore non-ARRIVED message type: {msg_type}")
            return

        ws, task = self._extract_ws_task(parsed)
        if ws is None or task is None:
            self._handle_unexpected(f"missing_ws_or_task:{raw}")
            return

        # Validate
        if self._allowed_ws and ws not in self._allowed_ws:
            self._handle_unexpected(f"unknown_ws:{ws}")
            return

        if self._allowed_tasks and task not in self._allowed_tasks:
            self._handle_unexpected(f"unknown_task:{task}")
            return

        # Busy handling
        if self._state != FSMState.IDLE:
            if self._busy_policy == "reject":
                self.get_logger().warn(f"Busy; reject ARRIVED {ws},{task}")
                self._send_fail_if_enabled(ws, task, "BUSY")
            else:
                self.get_logger().warn(f"Busy; ignore ARRIVED {ws},{task}")
            return

        # Trigger local manipulation
        self._pending_ws = ws
        self._pending_task = task
        self._pending_since = time.time()
        self._set_state(FSMState.WAIT_MANIP_RESULT)

        cmd = String()
        cmd.data = f"{ws},{task}"  # 로컬 명령 포맷(단순): "WSx,TASK"
        self._manip_cmd_pub.publish(cmd)
        self.get_logger().info(f"[MANIP_CMD] {cmd.data}")

    def _on_manip_result(self, msg: String):
        """
        로컬 매니퓰레이터 결과 수신.
        권장 포맷(간단):
          - "DONE"
          - "FAIL,REASON"
        또는 comm.md 스타일도 허용:
          - "DONE,WSx,TASK"
          - "FAIL,WSx,TASK,REASON"
        """
        raw = (msg.data or "").strip()
        if not raw:
            return

        if self._state != FSMState.WAIT_MANIP_RESULT or self._pending_ws is None or self._pending_task is None:
            self.get_logger().warn(f"Manip result received but no pending task: {raw}")
            return

        ws = self._pending_ws
        task = self._pending_task

        # 1) comm.md 스타일 우선 파싱
        parsed = parse_comm_md(raw, allow_legacy=True)
        if parsed and parsed.msg_type in [self._done_type, self._fail_type]:
            # ws/task가 없으면 pending 사용
            ws2 = parsed.ws or ws
            task2 = parsed.task or task

            if parsed.msg_type == self._done_type:
                self._send_done(ws2, task2)
                self._reset_pending()
                return

            if parsed.msg_type == self._fail_type:
                reason = parsed.reason or "FAIL"
                self._send_fail_if_enabled(ws2, task2, reason)
                self._reset_pending()
                return

        # 2) 간단 포맷 처리
        parts = _split_csv(raw)
        head = parts[0].upper() if parts else raw.upper()

        if head == "DONE":
            self._send_done(ws, task)
            self._reset_pending()
            return

        if head == "FAIL":
            reason = parts[1] if len(parts) >= 2 else "FAIL"
            self._send_fail_if_enabled(ws, task, reason)
            self._reset_pending()
            return

        # 3) 의도하지 않은 결과
        self.get_logger().warn(f"Unknown manip_result format: {raw}")
        self._send_fail_if_enabled(ws, task, "BAD_RESULT")
        self._reset_pending()

    def _on_tick(self):
        # Timeout watchdog
        if self._state != FSMState.WAIT_MANIP_RESULT:
            return
        if self._pending_ws is None or self._pending_task is None:
            return
        if self._timeout_sec <= 0:
            return

        if (time.time() - self._pending_since) > self._timeout_sec:
            ws, task = self._pending_ws, self._pending_task
            self.get_logger().warn(f"Manip timeout: {ws},{task} (>{self._timeout_sec:.1f}s)")
            self._send_fail_if_enabled(ws, task, "TIMEOUT")
            self._reset_pending()

    # -------------------------
    # Internals
    # -------------------------
    def _extract_ws_task(self, parsed: CommMsg) -> Tuple[Optional[str], Optional[str]]:
        # comm.md 기본: ws/task가 명시됨
        if parsed.ws and parsed.task:
            return parsed.ws, parsed.task

        # 레거시: ARRIVED:WS1_PICK3 같은 경우 ws에 "WS1_PICK3"만 들어올 수 있음
        if parsed.ws and not parsed.task:
            name = parsed.ws.strip()
            # 패턴: "WS1_PICK3" 또는 "WS1-PLACE3"
            for sep in ["_", "-"]:
                if sep in name:
                    a, b = [p.strip() for p in name.split(sep, 1)]
                    if a and b:
                        return a, b
            # ws만 있고 task 없으면 불완전
            return name, None

        return None, None

    def _send_done(self, ws: str, task: str):
        out = String()
        out.data = format_comm_csv(self._done_type, ws, task)
        self._serial_tx_pub.publish(out)
        self.get_logger().info(f"[SERIAL_TX] {out.data}")

    def _send_fail_if_enabled(self, ws: str, task: str, reason: str):
        if not self._enable_fail_tx:
            self.get_logger().warn(f"FAIL TX disabled; drop FAIL ({ws},{task},{reason})")
            return
        out = String()
        out.data = format_comm_csv(self._fail_type, ws, task, reason)
        self._serial_tx_pub.publish(out)
        self.get_logger().info(f"[SERIAL_TX] {out.data}")

    def _handle_unexpected(self, why: str):
        # 의도하지 않은 명령 예외 처리:
        # - 상태를 망치지 않고 로그만 남김 (필요 시 enable_fail_tx + busy_policy=reject 조합으로 적극 응답 가능)
        self.get_logger().warn(f"Unexpected serial command: {why}")

    def _set_state(self, st: FSMState):
        self._state = st
        if self._log_state:
            self.get_logger().info(f"[FSM] state={self._state}")

    def _reset_pending(self):
        self._pending_ws = None
        self._pending_task = None
        self._pending_since = 0.0
        self._set_state(FSMState.IDLE)

    def destroy_node(self):
        # 안전 종료
        self._reset_pending()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
