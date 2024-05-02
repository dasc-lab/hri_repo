import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket 
class SocketNode(Node):

    def __init__(self):
        super().__init__('socket_node')


        ############################## initialize socket ##############################
        self.HOST = '192.168.1.134'  # Server IP address
        self.PORT = 9999
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)
        print(f"Server listening on {self.HOST}:{self.PORT}")
        self.client_socket, self.addr = self.server_socket.accept()
        print(f"Connected to client: {self.addr}")

        ############################## create publisher ##############################
        self.publisher_ = self.create_publisher(String, '/socket/msgs', 10)
        timer_period = 1./100.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        data = self.client_socket.recv(1024)
        if data is not None:
            print("data")
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)

    socket_node = SocketNode()

    rclpy.spin(socket_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    socket_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()