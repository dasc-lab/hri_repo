import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener

import tf2_ros
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera

class MinimalSubscriber(Node):
   
	def __init__(self):
		
		self.quat = None
		self.trans = None
		self.robot_quat = None
		self.robot_trans = None
		super().__init__('minimal_subscriber')
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
		#self.subscription  # prevent unused variable warning
		
	def listener_callback_robot(self, msg):
	    self.robot_quat = msg.transform.rotation
	    self.robot_trans = msg.transform.translation
	    
	    #self.get_logger().info('I heard: "%s"' % msg.data)

	def listener_callback_camera(self, msg):
	    self.quat = msg.transform.rotation
	    self.trans = msg.transform.translation
	    #print("camera_info received")
	    #self.get_logger().info('I heard: "%s"' % msg.data)
	        
def main(args=None):
	
	rclpy.init(args=args)
	pipe = rs.pipeline()
	cfg = rs.config()
	cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
	cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

	c = pipe.start(cfg)
	profile = c.get_stream(rs.stream.color)
	color_intr = profile.as_video_stream_profile().get_intrinsics()

		#rclpy.spin(minimal_subscriber)
	minimal_subscriber = MinimalSubscriber()
	rclpy.spin_once(minimal_subscriber)
	while True:
		
		
		robot_trans = minimal_subscriber.robot_trans
		robot_quat = minimal_subscriber.robot_quat
		camera_trans = minimal_subscriber.trans
		camera_quat = minimal_subscriber.quat
		
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
		pixel_coordinates = np.floor(rs.rs2_project_point_to_pixel(color_intr, camera_coordinates))
		print(pixel_coordinates)
		frame = pipe.wait_for_frames()
		depth_frame = frame.get_depth_frame()
		color_frame = frame.get_color_frame()

		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		#color_intrin = color_frame.as_video_stream_profile().intrinsics

		
		pixel_coordinates = (int(pixel_coordinates[0]), int(pixel_coordinates[1]))
		color_image = cv2.circle(color_image, pixel_coordinates, 20, 1, 2)
		cv2.imshow('rgb', color_image)
		cv2.imshow('depth', depth_image)
		if cv2.waitKey(1) == 115:
		#cv.imwrite(str(device)+'_aligned_depth.png', depth_image)
			cv2.imwrite(str(device)+'_aligned_color.png',color_image)
			print("save image to directory")
		if cv2.waitKey(1) == ord('q'):
			break
	pipe.stop()
	minimal_subscriber.destroy_node()
	rclpy.shutdown()





	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()

	rclpy.shutdown()


if __name__ == '__main__':
    main()
