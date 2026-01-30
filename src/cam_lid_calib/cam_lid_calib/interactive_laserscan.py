import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
import numpy as np

class InteractiveLaserScan(Node):
    def __init__(self):
        super().__init__('interactive_laserscan')

        # LaserScan 토픽 구독
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/rplidar1/scan_filtered',
            self.laser_callback,
            10)

        # Interactive Marker 서버 설정
        self.server = InteractiveMarkerServer(self, "interactive_marker")

        # 기본 Interactive Marker 생성
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "lidar1_link"
        int_marker.name = "my_marker"
        int_marker.description = "Interactive Marker"
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0

        # Control 추가
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.orientation.w = float(1)
        control.orientation.x = float(1)
        control.orientation.y = float(0)
        control.orientation.z = float(0)
        control.always_visible = True
        int_marker.controls.append(control)

        box_marker = self.make_box()
        control.markers.append(box_marker)
        int_marker.controls.append(control)

        # x축 이동 제어 추가
        move_x_control = InteractiveMarkerControl()
        move_x_control.name = "move_x"
        move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_x_control.orientation.w = float(1)
        move_x_control.orientation.x = float(1)
        move_x_control.orientation.y = float(0)
        move_x_control.orientation.z = float(0)
        int_marker.controls.append(move_x_control)

        # y축 이동 제어 추가
        move_y_control = InteractiveMarkerControl()
        move_y_control.name = "move_y"
        move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_y_control.orientation.w = float(1)
        move_y_control.orientation.x = float(0)
        move_y_control.orientation.y = float(1)
        move_y_control.orientation.z = float(0)
        int_marker.controls.append(move_y_control)

        # z축 이동 제어 추가
        move_z_control = InteractiveMarkerControl()
        move_z_control.name = "move_z"
        move_z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_z_control.orientation.w = float(1)
        move_z_control.orientation.x = float(0)
        move_z_control.orientation.y = float(0)
        move_z_control.orientation.z = float(1)
        int_marker.controls.append(move_z_control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

        self.laser_data = None
        self.angle_min = None
        self.angle_increment = None

    def make_box(self):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def process_feedback(self, feedback):
        if self.laser_data is None or self.angle_min is None or self.angle_increment is None:
            return

        x = feedback.pose.position.x
        y = feedback.pose.position.y
        z = feedback.pose.position.z
        clicked_point = np.array([x, y, z])

        closest_range = None
        closest_angle = None
        min_distance = float('inf')

        for i, range in enumerate(self.laser_data):
            if range < 0.01 or np.isnan(range):
                continue

            angle = self.angle_min + i * self.angle_increment
            laser_point = np.array([range * np.cos(angle), range * np.sin(angle), 0.0])
            distance = np.linalg.norm(clicked_point - laser_point)

            if distance < min_distance:
                min_distance = distance
                closest_range = range
                closest_angle = angle

        if closest_range is not None and closest_angle is not None:
            self.get_logger().info(f'Closest Range: {closest_range}, Angle: {closest_angle}')

def main(args=None):
    rclpy.init(args=args)
    interactive_laserscan = InteractiveLaserScan()
    rclpy.spin(interactive_laserscan)
    interactive_laserscan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
