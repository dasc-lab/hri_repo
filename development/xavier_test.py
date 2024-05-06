import rclpy
from rclpy.node import Node
from std_msgs.msg import *
import numpy as np
from geometry_msgs.msg import TransformStamped
import pyrealsense2 as rs
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
import socket
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import time
from stream_transform import rot_to_hom, world_to_drone,drone_to_camera


class MinimalSubscriber(Node):
   
	def __init__(self):
		## initializing the camera
		super().__init__('minimal_subscriber')
		#self.output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MPEG'), 30, (640,480))
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
		self.frame_limit = 120
		# Setup server
		self.HOST = '192.168.1.134'  # Server IP address
		self.PORT = 9999
		self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_socket.bind((self.HOST, self.PORT))
		self.server_socket.listen(1)
		self.circle_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)] #Blue, Green, Red, Orange, Yellow...
		self.pos_list = [() for _ in range(0, self.num_robot)]
		self.trails = [[] for _ in range(0, self.num_robot)]
		print(f"Server listening on {self.HOST}:{self.PORT}")

		self.client_socket, self.addr = self.server_socket.accept()
		print(f"Connected to client: {self.addr}")
		self.pipe = rs.pipeline()
		self.cfg = rs.config()
		self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
		self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)
		#self.cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8,30)
		#self.cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,30)

		self.c = self.pipe.start(self.cfg)
		self.profile = self.c.get_stream(rs.stream.color)
		self.color_intr = self.profile.as_video_stream_profile().get_intrinsics()
		#self.depth_intrin = depth_image.as_video_stream_profile().get_intrinsics()
		self.robot_subscription = self.create_subscription(
		    TransformStamped,
		    #'/vicon/kepler/kepler',
		    #'/vicon/kepler/kepler',
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
		self.image_time = time.time()
		self.transmit_time = time.time()
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
	# def create_px4_msg(self, world_coordinates):
		
	def create_TransformStamped_msg(self, pixel_coordinates):
		transform_stamped_msg = TransformStamped()
		transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
		transform_stamped_msg.header.frame_id = 'parent_frame'
		transform_stamped_msg.child_frame_id = 'child_frame'
		transform_stamped_msg.translation.x = pixel_coordinates[0]
		transform_stamped_msg.translation.y = pixel_coordinates[1]
		transform_stamped_msg.transform.translation.z = 0.0
		transform_stamped_msg.transform.rotation.x = 0.0
		transform_stamped_msg.transform.rotation.y = 0.0
		transform_stamped_msg.transform.rotation.z = 0.0
		transform_stamped_msg.transform.rotation.w = 1.0
		return transform_stamped_msg
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
	def timer_callback(self):
		#pdb.set_trace()
		
		frame = self.pipe.wait_for_frames()
		depth_frame = frame.get_depth_frame()
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
			robot_world_coord = np.array([robot_trans.x, robot_trans.y, robot_trans.z, 1])
			hom_matrix_camera = np.array([0,0,0,0])
			
			drone_coordinates = world_to_drone(hom_matrix_camera, robot_world_coord)
			camera_coordinates = drone_to_camera(drone_coordinates)
			#coordinates = (-0.2,-0.1,3)
			pixel_coordinates = np.floor(rs.rs2_project_point_to_pixel(self.color_intr, camera_coordinates))
			print(pixel_coordinates)
			

			depth_image = np.asanyarray(depth_frame.get_data())
			depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha = 0.5), cv2.COLORMAP_JET)
			color_image = np.asanyarray(color_frame.get_data())
			
			
			#depth = depth_frame.get_distance(100,100)
			#depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [100,100], depth)
			
			pixel_coordinates = (int(pixel_coordinates[0]), int(pixel_coordinates[1]))
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
				    for j in range(1, len(self.trails[i])):
				    	if (not (j%4==0)):
				        	cv2.line(color_image, self.trails[i][j - 1], self.trails[i][j], self.circle_colors[i], thickness=2)

				# if ((time.time() - self.start_time) > 5):
				#     for trail in self.trails:
				#         if len(trail) > self.remove_index:
				#             del trail[:self.remove_index]

				if ((time.time() - self.start_time) > 5) :
				    if (len(trail) > self.remove_index for trail in self.trails):
				        if (time.time() - self.curr_time > 3):
				        	self.curr_time = time.time()
				        	#self.trails = [trail[self.remove_index:] for trail in self.trails]
				        	self.trails[i] = self.trails[i][30:]
				# Draw the current position of the circle
				if pos != (None,None):
				    self.draw_circle(color_image, (int(pos[0]), int(pos[1])), self.radius, self.circle_colors[i])
			image_time = time.time()-self.image_time
        	# Encode the frame
			cv2.imshow('rgb', color_image)
			cv2.imshow('depth', depth_cm)
			self.transmit_time = time.time()
			#self.output.write(color_image)
			_, buffer = cv2.imencode('.jpg', color_image)

			# Send the size of the frame and then the frame
			frame_size = len(buffer)
			size_buffer = frame_size.to_bytes(4, 'big')
			self.client_socket.sendall(size_buffer)
			self.client_socket.sendall(buffer.tobytes())
			transmit_time = time.time() - self.transmit_time
			print('image processing time: ' + str(image_time) + 'transmit time: ' + str(transmit_time))
			# Break the loop if 'q' is pressed
			if cv2.waitKey(1) & 0xFF == ord('q'):
			    rclpy.shutdown()
			if cv2.waitKey(1) == 115:
			#cv.imwrite(str(device)+'_aligned_depth.png', depth_image)
				cv2.imwrite('_aligned_color.png',color_image)
				print("save image to directory")
				
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

	rclpy.shutdown()
	minimal_subscriber.pipe.stop()

if __name__ == '__main__':
    main()
