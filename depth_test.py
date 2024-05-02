import pyrealsense2 as rs
import numpy as np
import cv2
import time

pipe = rs.pipeline()
cfg = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)
point = (int(0),int(0))
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
distance = 0.0
#point_list
start_time  = time.time()
def mouse_click(event, x, y,  
                flags, param):
    global point, point_list
    # to check if left mouse  
    # button was clicked 
    #print(x_stream, y_stream)
    if event == cv2.EVENT_LBUTTONDOWN: 
       point = (int(x),int(y))
       #np.append(point_list, point)
        # font for left click event 
        #cv2.putText(color_image, str(aligned_depth_frame.get_distance(int(x_stream),int(y_stream))), (x_stream,y_stream), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,255,255),2)
pipe.start(cfg)
cv2.namedWindow("color_image")
cv2.setMouseCallback("color_image",mouse_click)
frame = pipe.wait_for_frames()
#aligned_frames = align.process(frame)
#aligned_depth_frame = aligned_frames.get_depth_frame()
depth_frame = frame.get_depth_frame()
color_frame = frame.get_color_frame()
while True:
    frame = pipe.wait_for_frames()
    #aligned_frames = align.process(frame)
    #aligned_depth_frame = aligned_frames.get_depth_frame()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    #aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    print("(" + str(point[0]) + "," + str(point[1]) + ")")
    print(depth_frame.get_distance(point[0], point[1]))
    if (depth_frame.get_distance(point[0], point[1])>0.2):
        distance = depth_frame.get_distance(point[0], point[1])
    #cv2.putText(color_image, "%.2f" % depth_frame.get_distance(point[0], point[1]), point, cv2.FONT_HERSHEY_TRIPLEX, 1, (140,255,140),2)
    cv2.putText(color_image, "%.2f" % distance, point, cv2.FONT_HERSHEY_TRIPLEX, 1, (140,255,140),2)
    start_time = time.time()
    
    #if x_stream is not None and y_stream is not None:
    #cv2.putText(color_image, str(aligned_depth_frame.get_distance(int(x_stream),int(y_stream))), (x_stream,y_stream), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,255,255),2)
    cv2.imshow('color_image', color_image)

    cv2.imshow('depth', depth_image)
    #cv2.imshow('aligned image',aligned_depth_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
pipe.stop()

