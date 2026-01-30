import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageClickNode(Node):
    def __init__(self):
        super().__init__('image_click_node')

        # Image 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.bridge = CvBridge()
        self.cv_image = None

        # OpenCV 창 설정
        cv2.namedWindow('Camera')
        cv2.setMouseCallback('Camera', self.mouse_callback)

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Camera', self.cv_image)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f'Clicked at: x={x}, y={y}')

def main(args=None):
    rclpy.init(args=args)
    image_click_node = ImageClickNode()
    rclpy.spin(image_click_node)
    image_click_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
