import rclpy
import pyrealsense2 as rs
import numpy as np
import cv2

from rclpy.node import Node
from std_msgs.msg import *
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
import apriltag

pipe = rs.pipeline()
cfg = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

pipe.start(cfg)
options = apriltag.DetectorOptions(families="tag36h11")
#options = apriltag.DetectorOptions(families="tagStandard41h12")
detector = apriltag.Detector(options)
while True:
    frame = pipe.wait_for_frames()
    
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
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #print("[INFO] tag family: {}".format(tagFamily))
        print((cX,cY))
# show the output image after AprilTag detection
    cv2.imshow("Image", image)
    cv2.imshow('rgb', color_image)
    #cv2.imshow('depth', depth_image)
    #cv2.imshow('aligned image',aligned_depth_image)
    if cv2.waitKey(1) == ord('q'):
        break
pipe.stop()
