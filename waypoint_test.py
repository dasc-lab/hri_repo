import rclpy
from rover import MinimalPublisher
import socket



def main(args=None):
    HOST = '192.168.1.127'  # Server IP address
    PORT = 9997
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((self.HOST, self.PORT))
    server_socket.listen(1)
    print(f"Server listening on {self.HOST}:{self.PORT}")

    self.client_socket, self.addr = self.server_socket.accept()
    print(f"Connected to client: {self.addr}")

    while True:

        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()
        data = socket.recv(RECV_BUFFER).decode()
	    if data:
            minimal_publisher.pixel_coordinates = data
            rclpy.spin_once(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
