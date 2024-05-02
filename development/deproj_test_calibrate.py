import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import pdb
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import pyrealsense2 as rs
import numpy as np
import socket
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
import tf2_ros
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import time
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera, camera_to_world, world_to_camera, camera_to_world_calibrate
import matplotlib.pyplot as plt
import csv
class MinimalSubscriber(Node):
   
	def __init__(self):
		## initializing the camera
		super().__init__('minimal_subscriber')
		#self.output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MPEG'), 30, (640,480))
		self.quat = None
		self.trans = None
		self.robot_quat = None
		self.robot_trans = None
		self.max_error = 0
		self.distance_max_error_meter = 0
		self.distance_max_error_pixel = 0
		self.num_robot = 1
		self.radius = 20
		self.pixel_coordinates = None
		self.pixel_x = None
		self.pixel_y = None
		self.remove_index = 20

		# Setup server
		#self.HOST = '192.168.1.118'  # Server IP address
		#self.PORT = 9999
		#self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		#self.server_socket.bind((self.HOST, self.PORT))
		#self.server_socket.listen(1)
		self.circle_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)] #Blue, Green, Red, Orange, Yellow...
		self.pos_list = [() for _ in range(0, self.num_robot)]
		self.trails = [[] for _ in range(0, self.num_robot)]
		#print(f"Server listening on {self.HOST}:{self.PORT}")

		#self.client_socket, self.addr = self.server_socket.accept()
		#print(f"Connected to client: {self.addr}")
		self.pipe = rs.pipeline()
		self.cfg = rs.config()
		self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
		self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)
		#self.cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8,30)
		#self.cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,30)

		self.pixel_diff = []
		self.pixel_x_diff = []
		self.pixel_y_diff = []
		self.world_diff = []
		self.world_x_diff = []
		self.world_y_diff = []
		self.world_x_diff_origin = []
		self.world_y_diff_origin = []


		self.c = self.pipe.start(self.cfg)
		self.profile = self.c.get_stream(rs.stream.color)
		

		# self.depth_profile = self.c.get_stream(rs.stream.depth)
		# self.depth_sensor = self.depth_profile.get_device().first_depth_sensor()
		# self.depth_scale = self.depth_sensor.get_depth_scale()
		# print(depth_scale)
		self.depth_image = self.c.get_stream(rs.stream.depth)
		self.color_intr = self.profile.as_video_stream_profile().get_intrinsics()
		print(self.color_intr)
		self.depth_intrin = self.depth_image.as_video_stream_profile().get_intrinsics()
		self.robot_subscription = self.create_subscription(
		    TransformStamped,
		    '/vicon/px4_1/px4_1',
		    self.listener_callback_robot,
		    10)
		self.camera_subscription = self.create_subscription(
		    TransformStamped,
		    '/vicon/px4_1/px4_1',
		    self.listener_callback_camera,
		    10)  
		self.timer = self.create_timer(1./30., self.timer_callback)
		self.start_time = time.time()
		self.curr_time = time.time()
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
	
	def draw_circle(self, img, center, radius, color):
		cv2.circle(img, center, radius, color, thickness=2)  
	
	def draw_dot(self, img, center, radius=2, color =[255,0,0]):
		cv2.circle(img, center, radius, color, thickness=-1)
	


	def calculate_pixel_diff(self, pixel_coordinates):
		return np.sqrt((pixel_coordinates[0]-320)**2+(pixel_coordinates[1]-240)**2)
	
	def calculate_pixel_x_diff(self, pixel_coordinates):
		return np.absolute(pixel_coordinates[0]-320)

	def calculate_pixel_y_diff(self, pixel_coordinates):
		return np.absolute(pixel_coordinates[1]-240)

	def calculate_world_diff(self, vicon, predicted):
		return np.sqrt((vicon[0]-predicted[0])**2+(vicon[1]-predicted[1])**2)
	
	def calculate_world_x_diff(self, vicon, predicted):
		return np.absolute(vicon[0]-predicted[0])
	
	def calculate_world_y_diff(self, vicon, predicted):
		return np.absolute(vicon[1]-predicted[1])

	def calculate_world_x_diff_origin(self, vicon):
		return vicon[0]

	def calculate_world_y_diff_origin(self, vicon):
		return vicon[1]

# Function to send circle data to Unity
	
		
	def timer_callback(self):
		#pdb.set_trace()
		#print("timer callback")
		frame = self.pipe.wait_for_frames()
		depth_frame = frame.get_depth_frame().as_depth_frame()
		color_frame = frame.get_color_frame()

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
			robot_world_coord = np.array([robot_trans.x, robot_trans.y, robot_trans.z])
			hom_matrix_camera = np.array([0,0,0,0])
			
			#drone_coordinates = world_to_drone(hom_matrix_camera, robot_world_coord)
			camera_coordinates = world_to_camera(robot_world_coord)
			#coordinates = (-0.2,-0.1,3)
			pixel_coordinates = np.floor(rs.rs2_project_point_to_pixel(self.color_intr, camera_coordinates))
			
			

			depth_image = np.asanyarray(depth_frame.get_data())
			depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
			color_image = np.asanyarray(color_frame.get_data())
			
			
			#depth = depth_frame.get_distance(100,100)
			#depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [100,100], depth)
			
			pixel_coordinates = (int(pixel_coordinates[0]), int(pixel_coordinates[1]))
			
			x = max(pixel_coordinates[0],0)
			x = min(pixel_coordinates[0],640)
			y = max(pixel_coordinates[1],0)
			y = min(pixel_coordinates[1],480)
			
			depth = depth_frame.get_distance(pixel_coordinates[0],pixel_coordinates[1])
			predicted = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [pixel_coordinates[0],pixel_coordinates[1]], depth)
			predicted = camera_to_world_calibrate(predicted)
			text = 'pixel: '+ '(' + str(pixel_coordinates[0]) + ',' + str(pixel_coordinates[1]) + '),'
			text = text + 'predicted: '+ '(' + "%.2f" % predicted[0] + ',' + "%.2f" % predicted[1] + ',' + "%.2f" % predicted[2] +'),'
			text = text + 'Vicon: ' + '(' + "%.2f" % robot_trans.x + ',' + "%.2f" % robot_trans.y + ',' + "%.2f" % robot_trans.z + '),'
			text_coordinates = (pixel_coordinates[0]-100, pixel_coordinates[1]-5)
			cv2.putText(color_image, text, text_coordinates, cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,0,0),2)
			world_coordinate_error = self.calculate_world_diff((robot_trans.x,robot_trans.y,robot_trans.z),predicted)
			pixel_error = self.calculate_pixel_diff(pixel_coordinates)
			 
			if (world_coordinate_error > self.max_error):
				self.max_error = max(world_coordinate_error,self.max_error)
				self.distance_max_error_meter = self.calculate_world_diff((robot_trans.x,robot_trans.y,robot_trans.z),(0,0,0))
				self.distance_max_error_pixel = pixel_error
			#print(type(self.pixel_diff))
			vicon = (robot_trans.x,robot_trans.y,robot_trans.z)
			self.world_x_diff.append(self.calculate_world_x_diff(vicon,predicted))
			self.world_y_diff.append(self.calculate_world_y_diff(vicon, predicted))
			self.pixel_diff.append(self.calculate_pixel_diff(pixel_coordinates))
			self.world_diff.append(self.calculate_world_diff((robot_trans.x,robot_trans.y,robot_trans.z),predicted))
			self.world_x_diff_origin.append(self.calculate_world_x_diff_origin(vicon))
			self.world_y_diff_origin.append(self.calculate_world_y_diff_origin(vicon))


        	# Encode the frame
			cv2.imshow('rgb', color_image)
			cv2.imshow('depth', depth_cm)
			
			# Break the loop if 'q' is pressed
			if cv2.waitKey(1) & 0xFF == ord('q'):
			    rclpy.shutdown()
			if cv2.waitKey(1) == 115:
				#plt.plot(distance,error)
				print("key press")
				fields = ['distance_pxls','distance_meters']
				rows = np.array(list(zip(self.pixel_diff,self.world_diff)))
				print("start printing")
				plt.figure()
				plt.scatter(self.pixel_diff,self.world_diff)
				plt.xlabel("distance from origin (pxls)")
				plt.ylabel("Error between vicon and predicted (m)")
				plt.savefig("error_deproj1.png")
				plt.text(0.2,0.1, "distance from origin = %.2f" % self.distance_max_error_meter)
				plt.text(0.2,0.4, "distance from origin pixel = " + str(int(self.distance_max_error_pixel)))
				print("distance from origin at max error = %.2f" % self.distance_max_error_meter)
				print("distance from origin pixel = " + str(int(self.distance_max_error_pixel)))
				with open('world_discrepancy.csv','w') as file:
					# Create a CSV writer object that will write to the file 'file'
				    print("writing first csv file")
				    csv_writer = csv.writer(file)
				    
				    # Write the field names (column headers) to the first row of the CSV file
				    #csv_writer.writerow(fields)
				    
				    # Write all of the rows of data to the CSV file
				    csv_writer.writerows(rows)
				print("saving plot")
				
				plt.close()

				fields = ['distance_pxls','distance_x_meters']
				rows = np.array(list(zip(self.world_x_diff_origin,self.world_x_diff)))
				plt.figure()
				plt.scatter(self.world_x_diff_origin,self.world_x_diff)
				plt.xlabel("distance from origin (pxls)")
				plt.ylabel("Error between vicon and predicted x (m)")
				plt.savefig("error_deproj_x.png")
				#plt.text(0.2,0.1, "distance from origin = %.2f" % self.distance_max_error_meter)
				#plt.text(0.2,0.4, "distance from origin pixel = " + str(int(self.distance_max_error_pixel)))
				with open('world_discrepancy_x.csv','w') as file:
					print("writing second csv file")
					# Create a CSV writer object that will write to the file 'file'
					csv_writer = csv.writer(file)
				    
				    # Write the field names (column headers) to the first row of the CSV file
					#csv_writer.writerow(fields)

					# Write all of the rows of data to the CSV file
					csv_writer.writerows(rows)
				print("saving plot")

				fields = ['distance_pxls','distance_y_meters']
				rows = np.array(list(zip(self.world_y_diff_origin, self.world_y_diff)))
				plt.figure()
				plt.scatter(self.world_y_diff_origin, self.world_y_diff)
				plt.xlabel("distance from origin (pxls)")
				plt.ylabel("Error between vicon and predicted y (m)")
				plt.savefig("error_deproj_y.png")
				#plt.text(0.2,0.1, "distance from origin = %.2f" % self.distance_max_error_meter)
				#plt.text(0.2,0.4, "distance from origin pixel = " + str(int(self.distance_max_error_pixel)))
				#self.pipe.stop()
				with open('world_discrepancy_y.csv','w') as file:
					print("writing third csv file")
					# Create a CSV writer object that will write to the file 'file'
					csv_writer = csv.writer(file)

					# Write the field names (column headers) to the first row of the CSV file
					#csv_writer.writerow(fields)

					# Write all of the rows of data to the CSV file
					csv_writer.writerows(rows)
				print("saving plot")

def main(args=None):
	
	rclpy.init(args=args)

		#rclpy.spin(minimal_subscriber)
	minimal_subscriber = MinimalSubscriber()
	rclpy.spin(minimal_subscriber)
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()

	rclpy.shutdown()
	minimal_subscriber.pipe.stop()

if __name__ == '__main__':
    main()
