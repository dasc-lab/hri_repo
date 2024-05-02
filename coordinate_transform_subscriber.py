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

import tf2_ros
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import apriltag
from std_msgs.msg import String
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera
import matplotlib.pyplot as plt

error = []
distance = []
class MinimalSubscriber(Node):
   
	def __init__(self):
		## initializing the camera
		super().__init__('minimal_subscriber')
		self.quat = None
		self.trans = None
		self.robot_quat = None
		self.robot_trans = None

		
		self.pipe = rs.pipeline()
		self.cfg = rs.config()
		self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
		self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

		self.c = self.pipe.start(self.cfg)
		self.profile = self.c.get_stream(rs.stream.color)
		self.color_intr = self.profile.as_video_stream_profile().get_intrinsics()
		self.options = apriltag.DetectorOptions(families="tag36h11")
		self.detector = apriltag.Detector(self.options)

		self.robot_subscription = self.create_subscription(
		    TransformStamped,
		    #'/vicon/kepler/kepler',
		    '/vicon/teleop_rover/teleop_rover',
			self.listener_callback_robot,
		    10)
		self.camera_subscription = self.create_subscription(
		    TransformStamped,
		    '/vicon/px4_1/px4_1',
		    self.listener_callback_camera,
		    10)  
		self.timer = self.create_timer(1./30., self.timer_callback)
		#self.subscription  # prevent unused variable warning
	def pixel_distance(self, predicted, tag):
		return np.sqrt((predicted[0]-tag[0])**2+(predicted[1]-tag[1])**2)

	def listener_callback_robot(self, msg):
	    self.robot_quat = msg.transform.rotation
	    self.robot_trans = msg.transform.translation
	    
	    #self.get_logger().info('I heard: "%s"' % msg.data)

	def listener_callback_camera(self, msg):
	    self.quat = msg.transform.rotation
	    self.trans = msg.transform.translation
	    #print("camera_info received")
	    #self.get_logger().info('I heard: "%s"' % msg.data)

	def timer_callback(self):
		#pdb.set_trace()
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
			pixel_tuple = tuple(pixel_coordinates)
			# print(pixel_coordinates)
			frame = self.pipe.wait_for_frames()
			depth_frame = frame.get_depth_frame()
			color_frame = frame.get_color_frame()

			depth_image = np.asanyarray(depth_frame.get_data())
			depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
			color_image = np.asanyarray(color_frame.get_data())
			gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
			results = self.detector.detect(gray)
			image = color_image
			tag_center = (0,0)
			for r in results:
				(ptA, ptB, ptC, ptD) = r.corners
				ptB = (int(ptB[0]), int(ptB[1]))
				ptC = (int(ptC[0]), int(ptC[1]))
				ptD = (int(ptD[0]), int(ptD[1]))
				ptA = (int(ptA[0]), int(ptA[1]))
				center = (int((ptA[0]+ptB[0])/2),int((ptA[1]+ptC[1])/2))
				# draw the bounding box of the AprilTag detection
				cv2.line(image, ptA, ptB, (0, 255, 0), 2)
				cv2.line(image, ptB, ptC, (0, 255, 0), 2)
				cv2.line(image, ptC, ptD, (0, 255, 0), 2)
				cv2.line(image, ptD, ptA, (0, 255, 0), 2)
				# draw the center (x, y)-coordinates of the AprilTag
				(cX, cY) = (int(r.center[0]), int(r.center[1]))
				tag_center = (cX,cY)
				cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
			print(tag_center)
			#color_intrin = color_frame.as_video_stream_profile().intrinsics
			error.append(self.pixel_distance(pixel_tuple,tag_center))
			distance.append(self.pixel_distance(tag_center,(320,240)))
			color_image = image
			pixel_coordinates = (int(pixel_coordinates[0]), int(pixel_coordinates[1]))
			color_image = cv2.circle(color_image, pixel_coordinates, 20, 1, 2)
			cv2.imshow('rgb', color_image)
			cv2.imshow('depth', depth_cm)
			if cv2.waitKey(1) == 115:
				#plt.plot(distance,error)
				plt.scatter(distance,error)
				plt.xlabel("distance from origin")
				plt.ylabel("Error")
				plt.savefig("error_pixel.png")
			#cv.imwrite(str(device)+'_aligned_depth.png', depth_image)
				#cv2.imwrite('_aligned_color.png',color_image)
				print("save image to directory")
			if cv2.waitKey(1) == ord('q'):
				#minimal_subscriber.destroy_node()
				rclpy.shutdown()
			#self.pipe.stop()

def main(args=None):
	
	rclpy.init(args=args)
		#rclpy.spin(minimal_subscriber)
	minimal_subscriber = MinimalSubscriber()
	rclpy.spin(minimal_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	plt.plot(distance,error)
	plt.xlabel("distance from origin")
	plt.ylabel("Error between predicted pixel and actual pixel coordinates of the robot")
	plt.savefig("error_pixel.png")
	rclpy.shutdown()
	minimal_subscriber.pipe.stop()

if __name__ == '__main__':
    main()
