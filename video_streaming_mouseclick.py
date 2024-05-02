import pyrealsense2 as rs
import numpy as np
import cv2


pipe = rs.pipeline()
cfg = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)

clicked = False
endpoint = False

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)
#coordinates_list = []
#print(type(coordinates_list))
pipe.start(cfg)
#output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MPEG'), 30, (640,480))
def drawfunction(event,x,y,flags,param):
   if event == cv2.EVENT_LBUTTONDBLCLK:
      global clicked
      clicked = True
      global coordinates

      #placeholder = []
      #print(type(coordinates_list))
      print(clicked)
      print("x: ", x)
      print("y: ", y)
      coordinates = (tuple((x,y)))
      #print(type(placeholder))
      # print(placeholder)
      # for coordinates in placeholder:
        
      #      color_image = cv2.circle(color_image, coordinates, 4, (255,0,0),-1)
      # cv2.circle(color_image,(x,y),5,(255,0,0),-1)
      # cv2.imshow(color_image)


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
    #if coordinates_list and len(coordinates_list) > 0:
       #print("here")
       #for coordinates in coordinates_list:
    print(clicked)
    if clicked :
        print("here!")
        color_image = cv2.circle(color_image, coordinates, 4, (255,0,0),-1)
    #print(type(coordinates_list))
    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_image)
    cv2.imshow('aligned image',aligned_depth_image)
    key = cv2.waitKey(50)
    #output.write(color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
pipe.stop()
#output.release()
