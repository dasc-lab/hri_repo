import cv2
import apriltag
import rclpy
import pyrealsense2 as rs
import numpy as np
import px4_msgs_main
#variable declaration

def calibrate():
	extrinsics_matrix = None
	side = 0.6
	intrinsics_matrix = np.array([
								[376.746,   0,     324.664],
								[0,      376.362,  245.505],
								[0,         0,     1]
								])
	# intrinsics_matrix = np.array([
	# 							[3.76746,   0,     324.664],
	# 							[0,      3.76362,  245.505],
	# 							[0,         0,     1]
	# 							])
	world_coordinates = np.array([(2*side, 0, 0), (side,side,0), (-side,0,0), 
								(-side,side*2,0), (0,side,0), (2*side,-side,0),
								(side,-side,0),(2*side,side,0),(-side,-2*side,0.0)])
	# world_coordinates = np.array([(470, 0, 0), (235,235,0), (-235,0,0), 
	# 								(-235,470,0), (0,235,0), (470,-235,0),(235,-235,0),(470,235,0)])
	image_coordinates = np.array([])
	inverse_brandi = np.array([-0.0566381, 0.0674181, 0.000760374, 0.00101245, -0.0209169])
	#world_coordinates.append()
	pipe = rs.pipeline()
	cfg = rs.config()
	align_to = rs.stream.color
	align = rs.align(align_to)
	
	cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
	cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

	profile = pipe.start(cfg).get_stream(rs.stream.color)

	intr = profile.as_video_stream_profile().get_intrinsics()


	options = apriltag.DetectorOptions(families="tag36h11")
	#options = apriltag.DetectorOptions(families="tag41h12")
	counter = 0
	detector = apriltag.Detector(options)
	
	while counter <= 10:
	    frame = pipe.wait_for_frames()
	    tag_order = 0
	    #print("frame starts")
	    aligned_frames = align.process(frame)
	    aligned_depth_frame = aligned_frames.get_depth_frame()
	    depth_frame = frame.get_depth_frame()
	    color_frame = frame.get_color_frame()
	    aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
	    depth_image = np.asanyarray(depth_frame.get_data())
	    color_image = np.asanyarray(color_frame.get_data())
	    image = color_image
	    #print(type(image))
	    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	    results = detector.detect(gray)
	    for r in results:
	    	
	        # extract the bounding box (x, y)-coordinates for the AprilTag
	        # and convert each of the (x, y)-coordinate pairs to integers
	        #print("in the loop")
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
	        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
	        # draw the tag family on the image
	        tagFamily = r.tag_family.decode("utf-8")
	        center_string = '(' + str(cX) + ',' + str(cY) + ')'
	        cv2.putText(image, center_string, (ptA[0], ptA[1] - 15),
	            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
	        cv2.putText(image, 'ID: '+str(tag_order), (cX-20,cY), 
	        	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
	        tag_order = tag_order + 1
	        #print(tag_order)
	        #print("[INFO] tag family: {}".format(tagFamily))
	        #print((cX,cY))
	        if (counter == 0):
	        	
	        	image_coordinates = np.append(image_coordinates, (cX,cY))

	# show the output image after AprilTag detection
	    #cv2.imshow("Image", image)
	    #cv2.imshow('rgb', color_image)
	    counter = counter + 1
	    #cv2.imshow('depth', depth_image)
	    #cv2.imshow('aligned image',aligned_depth_image)
	    if cv2.waitKey(1) == ord('q'): # or counter > 10:
	        break
	#print(type(intr))
	#print(intrinsics_matrix)
	#print("world coordiantes", world_coordinates)
	#print("object points shape: ", len(world_coordinates))
	image_coordinates = image_coordinates.reshape((9,2))
	image_coordinates = image_coordinates.astype('float32')
	world_coordinates = world_coordinates.astype('float32')
	#print("image coordinates", image_coordinates)
	#print("image coordinates shape", len(image_coordinates))
	(success, rvec, tvec) = cv2.solvePnP(world_coordinates, image_coordinates, intrinsics_matrix,distCoeffs = 0,)
	rotate_matrix = cv2.Rodrigues(rvec)

	extrinsics_matrix = np.column_stack((rotate_matrix[0],tvec))
	extrinsics_matrix = np.row_stack((extrinsics_matrix, np.array([0,0,0,1])))
	return extrinsics_matrix
	#print(extrinsics_matrix)
	pipe.stop()
#mat = calibrate()
#print(mat)