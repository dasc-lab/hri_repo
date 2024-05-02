import numpy as np
import px4_msgs_main
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String
import socket

class MinimalPublisher(Node):

    def __init__(self):
        # super().__init__('minimal_publisher')
        # self.HOST = '192.168.1.134'  # Server IP address
        # self.PORT = 9999
        # self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_socket.bind((self.HOST, self.PORT))
        # self.server_socket.listen(1)
        # print(f"Server listening on {self.HOST}:{self.PORT}")

        # self.client_socket, self.addr = self.server_socket.accept()
        # print(f"Connected to client: {self.addr}")

        
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = self.create_TrajectorySetpoint_msg(np.array([-0.6,-0.6,-1.0]))
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing coordinates')
        self.i += 1
    def create_TrajectorySetpoint_msg(self, world_coordinates):
        msg = TrajectorySetpoint()
        #msg.raw_mode = False
        msg.position[0] = world_coordinates[0]
        msg.position[1] = world_coordinates[1]
        msg.position[2] = world_coordinates[2]
        #msg.yaw = (3.1415926 / 180.) * (float)(setpoint_yaw->value())
        msg.yaw = 0.0
        for i in range(3):
                msg.velocity[i] = 0.0
                msg.acceleration[i] = 0.0
                msg.jerk[i] = 0.0
        #msg.velocity = [0.2, 0.2, 0.2]
        msg.yawspeed = 0.0
        return msg
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin_once(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
