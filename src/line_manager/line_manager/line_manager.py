import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import time

class LineManager(Node):
    def __init__(self):
        super().__init__('line_manager')
        
        self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        self.create_subscription(String, '/control_from_amr', self.control_from_amr_callback, 10)
        self.pub_ = self.create_publisher(String, '/control_from_amr', 10) 
        self.amr_status_publisher = self.create_publisher(String, '/amr_status', 10)
        self.get_logger().info('line_manager node started')
        self.command = 0
        self.processes = {}
        # ch2(line_charge_out_node)
        # d2(destination2)
        # lu1(line_lift_up_node)
        # lqr(check_node)
        # lu2(line_lift_up_out_node)
        # d3(destination3)
        # ld1(line_lift_down_node)  
        # d1(destination1)
        # ch1(line_charge_node)
        self.amr_status = 'charging' # moving, loading, complete, charging
        self.publish_amr_status()

    def control_from_center_callback(self, msg):
        self.get_logger().info('controll_from_center_callback!!')

        if msg.data == 'shipment_start':
            self.command = 1 #ship =1
            self.start_nodes()

        elif msg.data == 'receivement_start':
            self.command = 2 #rec =2
            self.start_nodes()
    
        
        elif msg.data == 'ch2_start':
            self.amr_status = 'moving_to_load'
            self.publish_amr_status()
        
        elif msg.data == 'lu1_start':
            self.amr_status = 'loading'
            self.publish_amr_status()
        
        elif msg.data == 'd3_start':
            self.amr_status = 'moving_to_unload'
            self.publish_amr_status()

        elif msg.data == 'ld1_start':
            self.amr_status = 'unloading'
            self.publish_amr_status()
        
        elif msg.data == 'd1_start':
            self.amr_status = 'moving_to_charge'
            self.publish_amr_status()

        elif msg.data == 'kill':
            self.shutdown_nodes()
            self.amr_status = 'emergency_stop'                
            self.publish_amr_status()

        
    def control_from_amr_callback(self, msg):
        if msg.data == 'lu2_done':
            self.amr_status = 'load_complete'
            self.publish_amr_status()

        elif msg.data == 'ld1_done':
            self.amr_status = 'unload_complete'
            self.publish_amr_status()

        elif msg.data == 'ch1_done':
            self.shutdown_nodes()
            self.amr_status = 'charging'
            self.publish_amr_status()

        

    def start_nodes(self):
        self.get_logger().info(f'self.processes = {self.processes}')
        # print(self.processes)
        if not self.processes:
            
            self.processes['line_charge_out_node'] = subprocess.Popen(['ros2', 'run', 'line_follower', 'line_charge_out_node'])
            self.processes['line_lift_up_node'] = subprocess.Popen(['ros2', 'run', 'line_follower', 'line_lift_up_node'])
            self.processes['line_lift_up_out_node'] = subprocess.Popen(['ros2', 'run', 'line_follower', 'line_lift_up_out_node'])
            self.processes['line_lift_down_node'] = subprocess.Popen(['ros2', 'run', 'line_follower', 'line_lift_down_node'])
            self.processes['check_node'] = subprocess.Popen(['ros2', 'run', 'check_leg', 'check_node'])
            self.processes['line_charge_node'] = subprocess.Popen(['ros2', 'run', 'line_follower', 'line_charge_node'])
            time.sleep(1)
            self.get_logger().info('All nodes started')
            data = String()
            if self.command == 1:
                data.data = 'shipment_done'
            
            elif self.command == 2:
                data.data = 'receivement_done'
            
            
            self.pub_.publish(data)
        

    def shutdown_nodes(self):
        if self.processes:
            for name, process in list(self.processes.items()):
                if process.poll() is None:
                    self.get_logger().info(f'Shutting down {name}')
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warning(f'{name} did not terminate in time, killing it')
                        process.kill()
                        process.wait()
                    self.processes.pop(name)
            self.get_logger().info('All nodes shut down')
            time.sleep(2)
        self.processes = {}
        print('11111' ,self.processes)    

    def publish_amr_status(self):
        msg = String()
        msg.data = self.amr_status
        self.amr_status_publisher.publish(msg)
        self.get_logger().info(f'Published AMR status: {self.amr_status}')


def main(args=None):
    rclpy.init(args=args)
    node = LineManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()