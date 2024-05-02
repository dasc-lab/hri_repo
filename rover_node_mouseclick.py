import numpy as np
from pyrealsense2 import pyrealsense2 as rs
import px4_msgs_main
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String
from stream_transform import camera_to_world_calibrate
import socket
import apriltag
import cv2

clicked = False
endpoint = False
#coordinates = ()
def drawfunction(event,x,y,flags,param):
   if event == cv2.EVENT_LBUTTONDBLCLK:
      global clicked
      clicked = True
      global coordinates
      # print(clicked)
      # print("x: ", x)
      # print("y: ", y)
      coordinates = (tuple((x,y)))

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.HOST = '192.168.1.134'  # Server IP address
        # self.PORT = 9999
        # self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_socket.bind((self.HOST, self.PORT))
        # self.server_socket.listen(1)
        # print(f"Server listening on {self.HOST}:{self.PORT}")

        # self.client_socket, self.addr = self.server_socket.accept()
        # print(f"Connected to client: {self.addr}")

        self.pixel_coordinates = (200, 160)
        self.pipe = None
        self.cfg = None
        #self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
        #self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

        #self.c = self.pipe.start(self.cfg)
        self.profile = None
        self.depth_frame = None
        self.color_intr = None
        self.depth_intrin = None

        # self.pipe = rs.pipeline()
        # self.cfg = rs.config()
        # self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
        # self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

        # self.c = self.pipe.start(self.cfg)
        # self.profile = self.c.get_stream(rs.stream.color)
        # self.depth_image = self.c.get_stream(rs.stream.depth)
        # self.color_intr = self.profile.as_video_stream_profile().get_intrinsics()
        # self.depth_intrin = self.depth_image.as_video_stream_profile().get_intrinsics()


        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        self.timer_period = 1./30.  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # frame = self.pipe.wait_for_frames()
        # depth_frame = frame.get_depth_frame().as_depth_frame()
        # depth_image = np.asanyarray(depth_frame.get_data())
        # depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
        # color_frame = frame.get_color_frame()
        # color_image = np.asanyarray(color_frame.get_data())
        
        # #depth = depth_frame.get_distance(pixel_coordinates[0],pixel_coordinates[1])
        # cv2.imshow('rgb', color_image)
        # cv2.imshow('depth', depth_cm)
        
        msg = self.create_TrajectorySetpoint_msg(self.pixel_to_world_trans(self.pixel_coordinates))

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing coordinates')
        self.i += 1

    def world_to_robot(self, world_coordinates):
        robot_coordinates = (world_coordinates[1], world_coordinates[0], -1 * world_coordinates[2])
        return robot_coordinates
    def pixel_to_world_trans(self, pixel_coordinates):

        # frame = self.pipe.wait_for_frames()
        # depth_frame = frame.get_depth_frame()
        # depth_image = np.asanyarray(depth_frame.get_data())
        # depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
        # color_frame = frame.get_color_frame()
        # color_image = np.asanyarray(color_frame.get_data())
        # cv2.namedWindow('rgb')
        # cv2.setMouseCallback('rgb', drawfunction)
        # if clicked:
        #     color_image = cv2.circle(color_image, coordinates, 6, (255,0,0),-1)
        # cv2.imshow('rgb', color_image)
        # cv2.imshow('depth', depth_cm)
        # key = cv2.waitKey(50)
        # #output.write(color_image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     rclpy.shutdown()
        depth = self.depth_frame.get_distance(pixel_coordinates[0],pixel_coordinates[1])
        
        x = max(pixel_coordinates[0],0)
        x = min(pixel_coordinates[0],640)
        y = max(pixel_coordinates[1],0)
        y = min(pixel_coordinates[1],480)
        predicted = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [pixel_coordinates[0],pixel_coordinates[1]], depth)
        predicted = camera_to_world_calibrate(predicted)
        world_coordinates = self.world_to_robot(predicted)
        return world_coordinates
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
    pipe = rs.pipeline()
    cfg = rs.config()
    align_to = rs.stream.color
    align = rs.align(align_to)

    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

    c = pipe.start(cfg)
    profile = c.get_stream(rs.stream.color)
    depth_image = c.get_stream(rs.stream.depth)
    color_intr = profile.as_video_stream_profile().get_intrinsics()
    depth_intrin = depth_image.as_video_stream_profile().get_intrinsics()
    minimal_publisher.depth_intrin = depth_intrin
    minimal_publisher.color_intr = color_intr
    while True:
        frame = pipe.wait_for_frames()
        aligned_frames = align.process(frame)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()
        aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        cv2.namedWindow('rgb')
        cv2.setMouseCallback('rgb', drawfunction)
        #print(coordinates_list)
        #color_image = cv2.circle(color_image, (200, 160), 4, (255,0,0),-1)
        
        if clicked:
            minimal_publisher.depth_frame = depth_frame
            minimal_publisher.pixel_coordinates = coordinates
            color_image = cv2.circle(color_image, coordinates, 4, (255,0,0),-1)
            #print(type(coordinates_list))
            rclpy.spin_once(minimal_publisher)
        #print(type(coordinates_list))
        cv2.imshow('rgb', color_image)
        cv2.imshow('depth', depth_image)
        cv2.imshow('aligned image',aligned_depth_image)
        key = cv2.waitKey(50)
        #output.write(color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    pipe.stop()
    
    #rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
