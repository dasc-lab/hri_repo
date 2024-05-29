import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import pdb
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import TrajectorySetpoint
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
import socket
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import time
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera, camera_to_world_calibrate
import apriltag
import ast
import threading
import keyboard
class MinimalSubscriber(Node):
   
	def __init__(self):
		## initializing the camera
		super().__init__('minimal_subscriber')
		###################################################################################################
		####################### Declare and Initialize Preliminary Parameters ######################
		###################################################################################################
		self.quat = None
		self.trans = None
		self.robot_quat = None
		self.robot_trans = None
		self.num_robot = 1
		self.radius = 20
		self.pixel_coordinates = None
		self.pixel_x = None
		self.pixel_y = None
		self.remove_index = 20
		self.frame_count = 0
		self.frame_limit = 60
		self.is_target_set = False
		self.target = None
		self.circle_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)] #Blue, Green, Red, Orange, Yellow...
		self.pos_list = [() for _ in range(0, self.num_robot)]
		self.trails = [[] for _ in range(0, self.num_robot)]
		self.is_published = False
		self.reset = False
		self.rover_at_target = False
		self.waypoints = [(100,400), (100,100),(300,200),(500,100),(500,400)]
		self.waypoints = [(500,100), (500,400),(300,200),(100,400),(100,100)]
		self.numbers = [1,2,3,4,5]
		self.waypoint_threshold = 0
		###################################################################################################
		##################### Setup server ######################
		###################################################################################################
		
		self.HOST = '192.168.1.118'  # Server IP address
		self.PORT = 9999
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_socket.bind((self.HOST, self.PORT))
		self.server_socket.listen(1)
		print(f"Server listening on {self.HOST}:{self.PORT}")
		self.client_socket, self.addr = self.server_socket.accept()
		print(f"Connected to client: {self.addr}")
	
		
		##################### Set up Camera Parameters #####################
		self.pipe = rs.pipeline()
		self.cfg = rs.config()
		self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
		self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)
		
		self.c = self.pipe.start(self.cfg)
		self.profile = self.c.get_stream(rs.stream.color)
		self.depth_image = self.c.get_stream(rs.stream.depth)
		self.color_intr = self.profile.as_video_stream_profile().get_intrinsics()
		self.depth_intrin = self.depth_image.as_video_stream_profile().get_intrinsics()
		###################################################################################################
		########### Create Subscriber and Publisher ##########
		###################################################################################################
		########### Subscribers ###########
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
		########### Publisher ###########
		self.publisher_ = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
		self.i = 0
		###################################################################################################
		########### Set up timers for testing ###########
		###################################################################################################
		self.timer = self.create_timer(1./50., self.timer_callback)
		#self.loop_timer = self.create_timer(1./1000., self.loop_callback)
		self.start_time = time.time()
		self.curr_time = time.time()
		self.image_time = time.time()
		self.transmit_time = time.time()
		#self.subscription  # prevent unused variable warning
	###################################################################################################
	###################### Set up Callbacks ######################
	###################################################################################################
	def listener_callback_robot(self, msg):
		self.robot_quat = msg.transform.rotation
		self.robot_trans = msg.transform.translation
	    
	    #self.get_logger().info('I heard: "%s"' % msg.data)

	def listener_callback_camera(self, msg):
		self.quat = msg.transform.rotation
		self.trans = msg.transform.translation
	    
		
	###################################################################################################
	###################### Set up Helpers ######################
	###################################################################################################
	def target_reached(self, target_pixel_coordinates, actual_pixel_coordinates):
		'''
		Verify if robot has reached user designated pixel coordinates
		'''
		def euclidean_distance( point1, point2):
			"""
			Calculate the Euclidean distance between two 2D points.

			Args:
			point1 (tuple): Coordinates of the first point in the form (x1, y1).
			point2 (tuple): Coordinates of the second point in the form (x2, y2).

			Returns:
			float: The Euclidean distance between the two points.
			"""
			x1, y1 = point1
			x2, y2 = point2
			return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
		if euclidean_distance(target_pixel_coordinates,actual_pixel_coordinates) < 40:
			return True
		else:
			return False
	
	def draw_circle(self, img, center, radius, color):
		cv2.circle(img, center, radius, color, thickness=2)  

# Function to send circle data to Unity
	def send_circle_data(client_socket, circles):
		
		radius = 2
		circle_data = ','.join(f"{x},{y},{radius}" for x, y in circles)
		circle_data_bytes = circle_data.encode('utf-8')
		client_socket.sendall(len(circle_data_bytes).to_bytes(4, 'big'))
		client_socket.sendall(circle_data_bytes)

	def in_range(new_pos, trail_pos):
		return (new_pos[0]>=trail_pos[0]-3 and new_pos[0]<= trail_pos[0] + 3) and (new_pos[1]>=trail_pos[1]-3 and new_pos[1]<= trail_pos[1] + 3)
	
	
	
	def pixel_to_world_trans(self, pixel_coordinates):
		def world_to_robot( world_coordinates):
			robot_coordinates = (world_coordinates[1], world_coordinates[0], -1 * world_coordinates[2])
			return robot_coordinates
		frame = self.pipe.wait_for_frames()
		depth_frame = frame.get_depth_frame()
		depth = depth_frame.get_distance(pixel_coordinates[0],pixel_coordinates[1])
		####### safety #######
		x = max(pixel_coordinates[0],0)
		x = min(pixel_coordinates[0],640)
		y = max(pixel_coordinates[1],0)
		y = min(pixel_coordinates[1],480)
		predicted = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [x,y], depth)
		predicted = camera_to_world_calibrate(predicted)
		world_coordinates = world_to_robot(predicted)
		return world_coordinates
	
	def draw_waypoints(self, img, waypoints_arr, num_arr):
		#waypoints = [(100,100),(100,400), (300,200),(500,100),(500,400)]
		font = cv2.FONT_HERSHEY_SIMPLEX
		thickness = 2
		fontscale = 3
		for waypoint in waypoints_arr:
			self.draw_circle(img, waypoint, self.radius,(0,0,255))
		assert len(num_arr) == len(waypoints_arr)
		for number,center in zip(num_arr, waypoints_arr):
			center = (center[0]-3, center[1]-2)
			cv2.putText(img, str(number), center, font, fontscale,  (0,0,255), thickness)
			
	#def loop_callback(self):
	def create_TrajectorySetpoint_msg(self, world_coordinates):

		'''
		Create a TrjactorySetpoint message to command the rover
		'''
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
	
	def publish_msg(self):
		msg = self.create_TrajectorySetpoint_msg(self.pixel_to_world_trans(self.target))
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing coordinates')
		self.i += 1

	def draw_straight_line(self, img, coordinates):
		
		for i in range(len(coordinates) - 1):
			cv2.line(img, coordinates[i], coordinates[i+1], (255,0,0), thickness=2)
		return img
	
	def draw_dotted_curve(self, img, coordinates, dot_spacing=8,curve_color=(255, 0, 0)):
		'''
		draw dotted curve for past trajectories
		'''
		dashed = 0
		# Draw dotted lines along the curve
		for i in range(len(coordinates) - 1):
			
			if dashed < dot_spacing:
				img = cv2.line(img, coordinates[i], coordinates[i+1], curve_color, thickness=2)
				dashed = dashed + 1
			elif dashed >= dot_spacing and dashed <dot_spacing*2:
				dashed = dashed + 1
			else:
				dashed = 0
		return img
	
	def timer_callback(self):		
		frame = self.pipe.wait_for_frames()
		depth_frame = frame.get_depth_frame()
		color_frame = frame.get_color_frame()
		
		robot_trans = self.robot_trans
		robot_quat = self.robot_quat
		if self.reset == True:
			self.trails = [[] for _ in range(0, self.num_robot)]
			self.is_target_set = False
			self.target = None
			self.waypoint_threshold = 0
			self.reset = False
		if self.is_target_set and self.target is not None and self.is_published is False:
			self.publish_msg()
			self.is_published = True
		# if data:
		# 	data = data.decode('utf-8')
		# 	data = ast.literal_eval(data)
		# 	self.is_target_set = True
		# 	self.target = data
		# if self.is_target_set and (self.target is not None):
			############## publish and make the rover move ##############
			# self.publish_msg()
		# camera_trans = self.trans
		# camera_quat = self.quat
		#print(robot_trans, robot_quat)
		# print(robot_quat, robot_trans)
		if not (not robot_trans or not robot_quat ):
			# print("inside loop")

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
			#print(pixel_coordinates)
			

			depth_image = np.asanyarray(depth_frame.get_data())
			depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
			color_image = np.asanyarray(color_frame.get_data())
			
			
			#depth = depth_frame.get_distance(100,100)
			#depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [100,100], depth)
			
			pixel_coordinates = (int(pixel_coordinates[0]), int(pixel_coordinates[1]))
			if self.target is not None and self.is_target_set and self.target_reached(pixel_coordinates, self.target):
				print("set target reached to true")
				self.rover_at_target = True
				self.is_target_set = False
				self.target = None
			self.image_time=time.time()
			for i in range(0, self.num_robot):
			# Update the position of the circle
				
				pos = pixel_coordinates
				
				# Append the new position to the trail
				if pos != (None,None):
					if (len(self.trails[i]) > 0  and pos==self.trails[i][-1]):
						if self.frame_count < self.frame_limit:
							self.trails[i].append((int(pos[0]), int(pos[1])))
							self.frame_count = self.frame_count + 1
					else:
						self.frame_count=0
						self.trails[i].append((int(pos[0]), int(pos[1])))
				# Draw the complete trajectory for this circle
				
				if (len(self.trails[i])>=2):
					color_image = self.draw_straight_line(color_image,self.trails[i])
				
				# if ((time.time() - self.start_time) > 20) :
				# 	if (len(trail) > self.remove_index for trail in self.trails):
				# 		if (time.time() - self.curr_time > 3):
				# 			self.curr_time = time.time()
				# 			#self.trails = [trail[self.remove_index:] for trail in self.trails]
				# 			self.trails[i] = self.trails[i][40:]
				# Draw the current position of the circle
				self.draw_waypoints(color_image, self.waypoints[self.waypoint_threshold:], self.numbers[self.waypoint_threshold:])
				if pos != (None,None):
					self.draw_circle(color_image, (int(pos[0]), int(pos[1])), self.radius, self.circle_colors[i])
				if self.is_target_set and (self.target is not None):
					cv2.line(color_image, pixel_coordinates, self.target, (255, 0, 0),thickness = 2)
			image_time = time.time()-self.image_time
        	# Encode the frame
			cv2.imshow('rgb', color_image)
			cv2.imshow('depth', depth_cm)
			self.transmit_time = time.time()
			color_image = cv2.flip(color_image, -1)
			_, buffer = cv2.imencode('.jpg', color_image)

			# Send the size of the frame and then the frame
			frame_size = len(buffer)
			size_buffer = frame_size.to_bytes(4, 'big')
			


			self.client_socket.sendall(size_buffer)
			self.client_socket.sendall(buffer.tobytes())
			
			transmit_time = time.time() - self.transmit_time
			#print('image processing time: ' + str(image_time) + 'transmit time: ' + str(transmit_time))
			# Break the loop if 'q' is pressed
			if cv2.waitKey(1) & 0xFF == ord('r'):
				self.reset = True
			if cv2.waitKey(1) & 0xFF == ord('q'):
				rclpy.shutdown()
			if cv2.waitKey(1) == 115:
				cv2.imwrite('_aligned_color.png',color_image)
				print("save image to directory")
				
				#self.pipe.stop()




# HOST = '192.168.1.134'  # Xavier's IP address
# VIDEO_PORT = 9999
# COORD_PORT = 9998
def coordinate_reception(xavier_node):
	#HOST = '192.168.1.134'  # Xavier's IP address
	HOST = xavier_node.HOST
	VIDEO_PORT = 9999
	COORD_PORT = 9996
	coord_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	coord_server_socket.bind((HOST, COORD_PORT))
	coord_server_socket.listen(1)
	print(f"Coordinates server listening on {HOST}:{COORD_PORT}")

	coord_client_socket, addr = coord_server_socket.accept()
	print(f"Connected to coordinates client: {addr}")

	try:
		while True:
			coord_data = coord_client_socket.recv(1024).decode('utf-8')
			if not coord_data:
				break
			if coord_data == 'reset':
				xavier_node.reset = True
			else:
				coord_data = ast.literal_eval(coord_data)
				xavier_node.target = coord_data
				xavier_node.is_target_set = True
				xavier_node.is_published = False
				print(f"Received coordinates: {coord_data}")
				while xavier_node.rover_at_target is False:
					time.sleep(1)
					#print("in the loop")
				print("afterloop")
				if xavier_node.rover_at_target is True:
					xavier_node.rover_at_target = False
					xavier_node.waypoint_threshold = xavier_node.waypoint_threshold + 1
					message = "done"
					coord_client_socket.sendall(message.encode('utf-8'))
			
	finally:
		coord_client_socket.close()
		coord_server_socket.close()



def main(args=None):
	
	rclpy.init(args=args)
	#minimal_publisher = MinimalPublisher()
		#rclpy.spin(minimal_subscriber)
	minimal_subscriber = MinimalSubscriber()
	socket_thread = threading.Thread(target=coordinate_reception, args=(minimal_subscriber,))
	socket_thread.start()
	# terminal_thread = threading.Thread(target=terminal_command_listener, args=(minimal_subscriber,))
    # terminal_thread.start()
	print("here")
	rclpy.spin(minimal_subscriber)
	socket_thread.join()
    # terminal_thread.join()
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()

	rclpy.shutdown()
	minimal_subscriber.pipe.stop()

if __name__ == '__main__':
    main()
