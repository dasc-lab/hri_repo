import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import pdb
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera
import tf2_ros
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.quat = None
        self.trans = None
        self.robot_quat = None
        self.robot_trans = None
        self.pixel_coordinates = []
        self.robot_subscription = self.create_subscription(
            TransformStamped,
            '/vicon/kepler/kepler',
            self.listener_callback_robot,
            10)
        self.camera_subscription = self.create_subscription(
            TransformStamped,
            '/vicon/px4_1/px4_1',
            self.listener_callback_camera,
            10)

        self.publisher_ = self.create_publisher(TransformStamped, 'pixel_coordinates_robot', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #self.pixel_coordinates = list((0,0))
    def listener_callback_robot(self, msg):
        self.robot_quat = msg.transform.rotation
        self.robot_trans = msg.transform.translation
        
        #self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_callback_camera(self, msg):
        self.quat = msg.transform.rotation
        self.trans = msg.transform.translation
    def timer_callback(self):
        #self.pixel_coordinates[0] = self.pixel_coordinates[0] + 1
        #self.pixel_coordinates[1] = self.pixel_coordinates[1] + 1
        robot_trans = self.robot_trans
        robot_quat = self.robot_quat
        camera_trans = self.trans
        camera_quat = self.quat
        if not (not robot_trans or not robot_quat ):
            #or not camera_trans or not camera_quat

            r_robot = R.from_quat(np.array([robot_quat.x, robot_quat.y, robot_quat.z,robot_quat.w]))
            #r_camera = R.from_quat(np.array([camera_quat.x, camera_quat.y, camera_quat.z, camera_quat.w]))
            
            rot_matrix_robot = r_robot.as_matrix()
            #rot_matrix_camera = r_camera.as_matrix()
            #hom_matrix_robot = rot_to_hom(rot_matrix_robot, robot_trans)

            #hom_matrix_camera = rot_to_hom(rot_matrix_camera, camera_trans)
            #robot_world_coord = robot_to_world(hom_matrix_robot, robot_trans)
            robot_world_coord = np.array([robot_trans.x, robot_trans.y, robot_trans.z, 1])
            hom_matrix_camera = np.array([0,0,0,0])
            
            drone_coordinates = world_to_drone(hom_matrix_camera, robot_world_coord)
            camera_coordinates = drone_to_camera(drone_coordinates)
            #coordinates = (-0.2,-0.1,3)
            pixel_coordinates = np.floor(rs.rs2_project_point_to_pixel(self.color_intr, camera_coordinates))
        self.publisher_.publish(self.create_TransformStamped_msg(self.pixel_coordinates))
        
    
    def create_TransformStamped_msg(self, pixel_coordinates):
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        transform_stamped_msg.header.frame_id = 'parent_frame'
        transform_stamped_msg.child_frame_id = 'child_frame'
        #print(type(pixel_coordinates[0]))
        transform_stamped_msg.transform.translation.x = float(pixel_coordinates[0])
        transform_stamped_msg.transform.translation.y = float(pixel_coordinates[1])
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = 0.0
        transform_stamped_msg.transform.rotation.y = 0.0
        transform_stamped_msg.transform.rotation.z = 0.0
        transform_stamped_msg.transform.rotation.w = 1.0
        return transform_stamped_msg

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
