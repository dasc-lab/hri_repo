import scipy
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from tf.transformations import *
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
r = R.from_quat([0,0,1,0.5])
rotate = r.as_matrix()
rs.rs2_project_point_to_pixel(color_intrin, rotate)
def rot_to_hom(rot_matrix, trans):
    hom_mat = np.zeros((4,4))
    hom_mat[:3,:3] = rot_matrix
    hom_mat[4,4] = 1
    hom_mat[0,3]=-trans[0]
    hom_mat[1,3] = -trans[1]
    hom_mat[2,3] = -trans[2]
    return hom_mat
def  world_to_drone(hom_mat, coord):
    drone_coord = hom_mat * coord
    return drone_coord

def drone_to_camera(drone_coord):
    coord = drone_coord[1:3]
    mat = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    camera_coord = np.mat_mul(mat,coord)
    return camera_coord