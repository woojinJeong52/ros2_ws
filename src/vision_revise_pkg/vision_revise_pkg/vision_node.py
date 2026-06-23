import os
import rclpy
from rclpy.node import Node
from arm_interfaces.srv import GetTargetPose
from vision_revise_pkg import INUVisionCall as ivc


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {'1', 'true', 'yes', 'on'}
    return bool(value)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('service_name', '/get_target_pose')
        self.declare_parameter('camera_mode', 'mid_50')
        self.declare_parameter('brick_search_mode', 'fine')
        self.declare_parameter('local_id', 0)
        self.declare_parameter('visualize_capture', False)
        self.declare_parameter('visualize_search', False)
        self.declare_parameter('debug_summary', True)
        self.declare_parameter('yolo_device', 'auto')

        self.service_name = self.get_parameter('service_name').value
        self.srv = self.create_service(GetTargetPose, self.service_name, self.get_pose_cb)

        yolo_device = self.get_parameter('yolo_device').value
        self.get_logger().info('[VISION_REVISE] VisionManager 로드 중...')
        self.vision = ivc.VisionManager(device=yolo_device)
        self.get_logger().info(
            '[VISION_REVISE] 시작 완료 | '
            f'service={self.service_name}, '
            f'camera_mode={self.get_parameter("camera_mode").value}, '
            f'brick_search_mode={self.get_parameter("brick_search_mode").value}, '
            f'local_id={self.get_parameter("local_id").value}, '
            f'yolo_device={yolo_device}'
        )


    def get_pose_cb(self, request, response):
        target_str = request.target_color.strip()
        camera_mode = self.get_parameter('camera_mode').value
        brick_search_mode = self.get_parameter('brick_search_mode').value
        local_id = int(self.get_parameter('local_id').value)
        visualize_capture = _as_bool(self.get_parameter('visualize_capture').value)
        visualize_search = _as_bool(self.get_parameter('visualize_search').value)
        debug_summary = _as_bool(self.get_parameter('debug_summary').value)

        self.get_logger().info(
            '[VISION_REVISE] 요청 수신 | '
            f'target_id={target_str}, camera_mode={camera_mode}, '
            f'brick_search_mode={brick_search_mode}, local_id={local_id}'
        )

        try:
            if not target_str.isdigit():
                self.get_logger().error(
                    f'[VISION_REVISE] 잘못된 입력입니다. 숫자 ID를 입력하세요: {target_str}'
                )
                response.success = False
                return response

            target_id = int(target_str)

            result = self.vision.run_pipeline_by_id(
                target_id=target_id,
                local_id=local_id,
                camera_mode=camera_mode,
                brick_search_mode=brick_search_mode,
                V_visualize_capture=visualize_capture,
                V_visualize_search=visualize_search
            )

            if debug_summary:
                self.get_logger().info(f'[VISION_REVISE] pipeline result={result}')

            if result["success"]:
                response.success = True

                # 내부 단위: mm
                # ROS 응답 단위: m
                response.x = float(result["x_mm"] / 1000.0)
                response.y = float(result["y_mm"] / 1000.0)
                response.z = float(result["z_mm"] / 1000.0)
                response.yaw = float(result["yaw_deg"] + 90.0)
                response.class_name = str(result["class_name"])

                self.get_logger().info(
                    f'[VISION_REVISE] 타겟 발견 | '
                    f'ID={result["target_id"]}, '
                    f'Class={result["class_name"]}, '
                    f'X={result["x_mm"]:.1f}mm, '
                    f'Y={result["y_mm"]:.1f}mm, '
                    f'Z={result["z_mm"]:.1f}mm, '
                    f'Yaw={result["yaw_deg"]:.2f}deg'
                )

            else:
                response.success = False
                self.get_logger().error(
                    f'[VISION_REVISE] 타겟 탐색 실패 | '
                    f'ID={result.get("target_id")}, '
                    f'Class={result.get("class_name")}, '
                    f'Reason={result.get("reason")}'
                )

        except Exception as e:
            self.get_logger().exception(f'[VISION_REVISE] 처리 중 심각한 오류 발생: {e}')
            response.success = False

        return response


def main(args=None):
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
