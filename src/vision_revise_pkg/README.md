# vision_revise_pkg

`vision_pkg`를 기반으로 디버깅과 반복 실행 안정성을 보강한 패키지입니다.

## Build

```bash
cd /home/moonshot/ros2_ws
colcon build --packages-select vision_revise_pkg
source install/setup.bash
```

## Run

기본 실행:

```bash
ros2 run vision_revise_pkg vision_node
```

디버그 launch 실행:

```bash
ros2 launch vision_revise_pkg vision_debug.launch.py
```

시각화까지 켜서 실행:

```bash
ros2 launch vision_revise_pkg vision_debug.launch.py \
  visualize_capture:=true \
  visualize_search:=true
```

CPU 강제 실행:

```bash
ros2 launch vision_revise_pkg vision_debug.launch.py yolo_device:=cpu
```

## Service Call

`target_color` 필드는 기존 인터페이스 이름을 유지하지만 실제 값은 숫자 ID 문자열입니다.

```bash
ros2 service call /get_target_pose arm_interfaces/srv/GetTargetPose \
  "{target_color: '1', target_size: ''}"
```

## Useful Parameters

- `service_name`: 서비스 이름. 기본값 `/get_target_pose`
- `camera_mode`: `floor`, `macro_30`, `mid_50`. 기본값 `mid_50`
- `brick_search_mode`: `coarse`, `fine`. 기본값 `fine`
- `local_id`: 같은 클래스가 여러 개 잡혔을 때 선택할 객체 index. 기본값 `0`
- `visualize_capture`: RealSense 캡처 시각화
- `visualize_search`: YOLO/depth 처리 시각화
- `debug_summary`: pipeline result dict 로그 출력
- `yolo_device`: `auto`, `cpu`, `0` 등. 기본값 `auto`

## Notes

- `pyrealsense2`나 `open3d`가 없는 환경에서도 노드 자체는 기동됩니다.
- 실제 서비스 호출 시 필요한 모듈이 없으면 ROS 로그에 명확한 의존성 오류가 출력됩니다.
- 제한된 환경에서도 ROS 로그를 쓸 수 있도록 `ROS_LOG_DIR` 기본값을 `/tmp/ros_logs`로 설정합니다.
