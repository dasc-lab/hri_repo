import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
from calibrate import calibrate
from std_msgs.msg import String

#extrinsics = calibrate()

extrinsics = np.array([[ 0.99926912, -0.03471002,  0.01601377, -0.34649297],
                        [-0.03404074, -0.9986065 , -0.04032727,  0.14374124],
                        [ 0.01739122,  0.03975268, -0.99905819,  2.76456235],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]])

def rot_to_hom(rot_matrix, trans):

    '''
    Construct Homogeneous matrix from R and T

    rot_matrix: (3x3) np.ndarray
    trans: (3x1) np.ndarray

    '''
    hom_mat = np.eye(4)
    hom_mat[:3,:3] = rot_matrix
    hom_mat[:3,-1] = trans
    return hom_mat


def robot_to_world(hom_mat, coord):

    return np.mat_mul(hom_mat, coord)
def camera_to_world(hom_mat, camera_coord):
    hom_mat = np.eye(4)
    hom_mat[1,1] = -1
    hom_mat[2,2] = -1
    z = 2.8
    x = 0.3117
    y = 0.191
    hom_mat[0,3] = x
    hom_mat[1,3] = y
    hom_mat[2,3] = z
    world_coordinates = hom_mat @ np.array([camera_coord[0], camera_coord[1],camera_coord[2],1])
    return world_coordinates[0:3]

def world_to_camera(world_coordinates):
    '''
    Transform point from world to camera coordinate system with calibrated extrinsincs matrix
    
    world coordinates: np.array 3x1 vector, the coordinate of a point in world coordinate system
    '''
    world_coordinates = np.append(world_coordinates,1)
    hom_mat = extrinsics
    camera_coord = hom_mat @ world_coordinates
    camera_coord = camera_coord[0:3]
    return camera_coord
def camera_to_world_calibrate(camera_coordinates):
    '''
    Transform point from camera to world coordinate system with calibrated extrinsincs matrix
    
    camera coordinates: np.array 3x1 vector, the coordinate of a point in camera coordinate system
    '''
    camera_coordinates = np.append(camera_coordinates,1)
    hom_mat = extrinsics
    hom_mat = np.linalg.inv(hom_mat)
    world_coordinates = hom_mat @ camera_coordinates
    return world_coordinates[0:3]

def  world_to_drone(hom_mat, coord):
    '''

    hom_mat: (4x4) np.ndarray
    coord  : (4x1) np.ndarray
    
    '''
    ############## for testing ################
    robot_height = 0.3
    z = 2.8
    x = 0.3117
    y = 0.191
    hom_mat2 = np.eye(4)
    hom_mat2[0,3] = -x
    hom_mat2[1,3] = -y
    hom_mat2[2,3] = -z

    ###########################################
    # hom_mat2 = inv(hom_mat)
    drone_coord = (hom_mat2) @ coord
    return drone_coord

def drone_to_camera(drone_coord):
    '''
        drone -> cam

        H = [R t]
            [0 1]

    '''
    coord = drone_coord[0:3]
    mat = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    camera_coord = np.matmul(mat,coord)
    return camera_coord

r = R.from_quat([0,0,1,0.5])
rotate = r.as_matrix()


