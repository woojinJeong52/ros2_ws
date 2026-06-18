# amr_robot_ws

> 버전: v1 | 최종 수정: 2026-06-10

RoboCup SML 프로젝트의 AMR 로봇팔 제어 워크스페이스입니다.
AMR 위에 탑재된 로봇팔의 LOAD / UNLOAD 동작을 제어하고,
슬롯 상태를 관리하는 시스템입니다.

---

## 📦 패키지 구조

```
amr_robot_ws/src/
  arm_interfaces/            # 공용 서비스 인터페이스 패키지
    srv/
      Cargo.srv              # 슬롯 상태 관리 인터페이스
      GetTargetPose.srv      # 비전 타겟 위치 인터페이스

  arm_controller_pkg/        # 로봇팔 제어 노드 패키지
    arm_controller_pkg/
      amr_robot_node.py      # LOAD / UNLOAD 통합 제어 노드
      cargo_manager_node.py  # AMR 슬롯 상태 관리 노드
      gripper_node.py        # 그리퍼 제어 노드

  vision_pkg/                # 비전 패키지 (물체 위치 탐지)

  amr_robot_launch/          # 전체 시스템 일괄 실행용 런치 패키지
    launch/
      amr_robot.launch.py    # vision/gripper/cargo/amr_robot 노드 동시 실행
```

---

## 🗂️ 노드 역할

| 노드 | 역할 |
|------|------|
| `amr_robot_node` | LOAD / UNLOAD 명령 수신 및 로봇팔 시퀀스 실행 |
| `cargo_manager_node` | AMR 슬롯 점유 상태 추적 및 관리 |
| `gripper_node` | 그리퍼 grip / open 제어 |

---

## 🔗 통신 구조

```
sml_manager_node (MasterPC_ws)
      ↓ Service /amr_robot_command
  amr_robot_node
      ├── Service /cargo        → cargo_manager_node
      ├── Service /gripper/grip → gripper_node
      ├── Service /gripper/open → gripper_node
      └── Service /get_target_pose → vision_pkg
```

---

## 📨 인터페이스 정의

### ArmCommand.srv (sml_manager_node 연동용)

```
# Request
string  action          # "LOAD" / "UNLOAD"
int32[] object_ids      # 처리할 물체 ID 리스트
string  location        # 미사용 (호환성 유지)

---
# Response
bool    success
int32[] slots           # 실제 처리된 슬롯 번호
int32[] object_ids      # 실제 처리된 물체 ID
string  message         # 실패 시 이유
```

**실패 메시지 종류:**

| message | 의미 | 대응 |
|---------|------|------|
| `no empty slot` | 슬롯 꽉 참 | 계획 오류 |
| `vision failed` | 카메라 인식 실패 | 재시도 가능 |
| `grip failed` | 그리퍼 실패 | 재시도 가능 |
| `object not found` | UNLOAD할 물체 없음 | 계획 오류 |
| `robot not connected` | 로봇 연결 안 됨 | 하드웨어 확인 |

---

## 🚗 AMR 슬롯 구조

| 슬롯 | 용도 | 제약 |
|------|------|------|
| 슬롯 1 | 완성품 / 분해 대상 전용 | 동시에 1개만 점유 가능 |
| 슬롯 2~6 | 재료 전용 | 최대 5개 |

슬롯 배정은 `amr_robot_node`가 `cargo_manager_node`를 통해 자동 처리합니다.

---

## 🔧 재료 ID 매핑

| ID | 재료 |
|----|------|
| 1 | 2x2_red |
| 2 | 2x2_green |
| 3 | 2x2_blue |
| 4 | 2x2_yellow |
| 5 | 4x2_red |
| 6 | 4x2_green |
| 7 | 4x2_blue |
| 8 | 4x2_yellow |

---

## ▶️ 실행 방법

터미널마다 아래 source를 먼저 실행하세요:

```bash
source /opt/ros/humble/setup.bash
source ~/robocup/amr_robot_ws/install/setup.bash
```

### 방법 A. 런치 파일로 한 번에 실행 (권장)

`amr_robot_launch` 패키지의 런치 파일이 아래 4개 노드를 한 터미널에서 모두 실행합니다:
`vision_node`, `gripper_node`, `cargo_manager_node`, `amr_robot_node`.

```bash
ros2 launch amr_robot_launch amr_robot.launch.py
```

현재 로봇 IP(`10.0.2.8`), 그리퍼 시리얼 포트(`/dev/ttyARDUINO`) 등은
코드에 하드코딩된 값을 그대로 사용하며, 런치 인자(argument)는 제공하지 않습니다.
값을 바꿔야 하면 각 노드 코드를 직접 수정한 뒤 다시 빌드하세요.

`Ctrl+C` 한 번으로 4개 노드가 모두 종료됩니다.

### 방법 B. 노드별로 따로 실행 (디버깅용)

개별 노드의 로그를 분리해서 보고 싶거나, 특정 노드만 재시작하고 싶을 때 사용하세요.

**터미널 1 — 비전 노드**
```bash
ros2 run vision_pkg vision_node
```

**터미널 2 — 그리퍼 노드**
```bash
ros2 run arm_controller_pkg gripper_node
```

**터미널 3 — 슬롯 관리 노드**
```bash
ros2 run arm_controller_pkg cargo_manager_node
```

**터미널 4 — 로봇팔 제어 노드**
```bash
ros2 run arm_controller_pkg amr_robot_node
```

**디버깅 — 슬롯 상태 확인**
```bash
ros2 service call /cargo arm_interfaces/srv/Cargo \
  "{action: 'STATUS', object_id: 0, slot: 0}"
```

**디버깅 — 직접 LOAD 명령 테스트**
```bash
ros2 service call /amr_robot_command arm_interfaces/srv/ArmCommand \
  "{action: 'LOAD', object_ids: [1], location: ''}"
```

---

