import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import time

GRIPPER_PORT = "/dev/ttyARDUINO"
GRIPPER_BAUD = 115200


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        try:
            self.ser = serial.Serial(GRIPPER_PORT, GRIPPER_BAUD, timeout=1)
            time.sleep(2)
            self.get_logger().info('[GRIPPER] serial connected')
            self.clear_serial_buffer()
            self.ser.write(b"open\n")
            self.ser.flush()
            self.get_logger().info('[GRIPPER] initialized: open')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'[GRIPPER] serial error: {e}')

        self.srv_open = self.create_service(Trigger, '/gripper/open', self.handle_open)
        self.srv_grip = self.create_service(Trigger, '/gripper/grip', self.handle_grip)
        self.get_logger().info('[GRIPPER] gripper_node started')

    def clear_serial_buffer(self):
        if self.ser is None:
            return
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            self.get_logger().warn(f'[GRIPPER] buffer clear failed: {e}')

    def wait_grasp_result(self, timeout=5.0):
        if self.ser is None:
            return False, 'Serial not connected'
        start = time.time()
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if '[GRASP] Condition satisfied. HOLD.' in line:
                    return True, 'GRASP_OK'
                elif '[GRASP] Torque remains ON.' in line:
                    return True, 'GRASP_OK'
                elif '[RESULT] GRASP_OK' in line:
                    return True, 'GRASP_OK'
                elif '[RESULT] GRASP_FAIL' in line:
                    return False, 'GRASP_FAIL'
            time.sleep(0.01)
        return False, 'Timeout'

    def handle_open(self, request, response):
        if self.ser is None:
            response.success = False
            response.message = 'Gripper not connected'
            return response
        try:
            self.clear_serial_buffer()
            self.ser.write(b"open\n")
            self.ser.flush()
            self.get_logger().info('[GRIPPER] open')
            time.sleep(1.0)
            response.success = True
            response.message = 'Gripper opened'
        except Exception as e:
            self.get_logger().error(f'[GRIPPER] open error: {e}')
            response.success = False
            response.message = str(e)
        return response

    def handle_grip(self, request, response):
        if self.ser is None:
            response.success = False
            response.message = 'Gripper not connected'
            return response
        try:
            self.clear_serial_buffer()
            self.ser.write(b"grip\n")
            self.ser.flush()
            self.get_logger().info('[GRIPPER] grip (waiting result...)')
            success, msg = self.wait_grasp_result(timeout=5.0)
            response.success = success
            response.message = msg
            if success:
                self.get_logger().info('[GRIPPER] GRASP_OK')
            else:
                self.get_logger().warn(f'[GRIPPER] grip failed: {msg}')
        except Exception as e:
            self.get_logger().error(f'[GRIPPER] grip error: {e}')
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('[GRIPPER] serial closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
