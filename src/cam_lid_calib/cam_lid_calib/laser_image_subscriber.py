import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaserImageSubscriber(Node):
    def __init__(self):
        super().__init__('laser_image_subscriber')

        # LaserScan 토픽 구독
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/rplidar1/scan_filtered',
            self.laser_callback,
            10)

        # Image 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        # Image 토픽 발행
        self.image_publisher = self.create_publisher(Image, '/camera/image_with_laserscan', 10)

        self.bridge = CvBridge()

        # 레이저 데이터 저장 변수
        self.laser_data = None
        self.angle_min = None
        self.angle_increment = None

    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.laser_data is not None and self.angle_min is not None and self.angle_increment is not None:
            self.plot_laserscan_on_image(cv_image, self.laser_data, self.angle_min, self.angle_increment)

        # 이미지를 Image 메시지로 변환하여 발행
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.image_publisher.publish(img_msg)

    def plot_laserscan_on_image(self, image, laser_data, angle_min, angle_increment):
        height, width, _ = image.shape

        for i, range in enumerate(laser_data):
            if range < 0.01 or np.isnan(range):  # 유효하지 않은 데이터는 무시
                continue

            angle = angle_min + i * angle_increment

            # 각도가 0 ~ π 범위 내에 있는지 확인
            if angle < 0:
                continue

            # 레이저 포인트 3D 좌표 (레이저 센서 프레임 기준)
            laser_point = np.array([[range * np.cos(angle)],
                                    [range * np.sin(angle)],
                                    [1]
            ])

            # Homography matrix H
            H = np.array([
                [0.762313374654778, 0.554501660487931, -0.116701863706915],
                [-0.012759675416174, 0.311968681559555, 0.017225532335290],
                [-3.361176241344821e-05, 0.001705005620124, -8.727555271565602e-05]
            ])
            
            # 이미지 프레임으로 변환
            cam_xyw = np.dot(H, laser_point)

            if np.isnan(cam_xyw[0]) or np.isnan(cam_xyw[1]):
                continue

            # 2D 이미지 평면으로 투영 (간단히 x, y 축만 사용)
            img_x = int(cam_xyw[0] / cam_xyw[2])
            img_y = int(cam_xyw[1] / cam_xyw[2])

            if 0 <= img_x < width and 0 <= img_y < height:
                cv2.circle(image, (img_x, img_y), 2, self.circle_color(range), -1)

    def circle_color(self, range):
        # Define the thresholds
        red_threshold = 1.0
        green_threshold = 2.0
        blue_threshold = 3.0
        purple_threshold = 4.0

        if range < red_threshold:
            color = (0, 0, 255)  # Red
        elif range >= red_threshold and range < green_threshold:
            # Transition from red to green
            ratio = (range - red_threshold) / (green_threshold - red_threshold)
            red = int(255 * (1 - ratio))
            green = int(255 * ratio)
            color = (0, green, red)
        elif range >= green_threshold and range < blue_threshold:
            # Transition from green to blue
            ratio = (range - green_threshold) / (blue_threshold - green_threshold)
            green = int(255 * (1 - ratio))
            blue = int(255 * ratio)
            color = (blue, green, 0)
        elif range >= blue_threshold and range < purple_threshold:
            # Transition from blue to purple
            ratio = (range - blue_threshold) / (purple_threshold - blue_threshold)
            blue = int(255 * (1 - ratio) + 128 * ratio)
            red = int(128 * ratio)
            color = (blue, 0, red)
        else:
            color = (255, 0, 255)  # Purple

        return color

def main(args=None):
    rclpy.init(args=args)
    laser_image_subscriber = LaserImageSubscriber()
    rclpy.spin(laser_image_subscriber)
    laser_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
