import pyrealsense2 as rs
import numpy as np
import cv2

def consecutive_column_differences(matrix):
    # Get the number of columns
    num_cols = matrix.shape[1]
    
    # Initialize an empty array to store the differences
    diff_matrix = np.zeros((matrix.shape[0], num_cols - 1))
    
    # Calculate differences between consecutive columns
    for i in range(0,num_cols-1):
        if i < num_cols // 2:  # Left half of the matrix
            diff_matrix[:, i] = matrix[:, i] - matrix[:, i+1]
        else:  # Right half of the matrix
            diff_matrix[:, i] = matrix[:, i+1] - matrix[:, i]
    if diff_matrix.shape[1]%2 != 0 :
        center_col_index = (num_cols-1) // 2
    
    # Discard the center column
        diff_matrix = np.delete(diff_matrix, center_col_index, axis=1)
    return diff_matrix
def column_averages(diff_matrix):
    # Calculate column-wise averages
    avg_vector = np.mean(diff_matrix, axis=0)
    return avg_vector
def column_difference_error(diff_vector,col_vector):
    col_vector = col_vector[1:-1]
    error_vec = diff_vector/col_vector
    return error_vec
def save_matrix_txt(matrix, text):
    np.savetxt(text, matrix)
def save(depth_image, aligned_depth_image):
    depth_difference = consecutive_column_differences(depth_image)
    depth_col = column_averages(depth_image)
    depth_column_diff_average = column_averages(depth_difference)
    depth_error = column_difference_error(depth_column_diff_average, depth_col)

    aligned_col = column_averages(aligned_depth_image)

    aligned_depth_difference = consecutive_column_differences(aligned_depth_image)
    aligned_depth_column_average = column_averages(aligned_depth_difference)
    aligned_error = column_difference_error(aligned_depth_column_average, aligned_col)

    save_matrix_txt(aligned_depth_difference, "aligned depth_difference_matrix.txt")
    save_matrix_txt(depth_difference, "depth_difference_matrix.txt")
    save_matrix_txt(depth_column_diff_average, "depth_vector.txt")
    save_matrix_txt(aligned_depth_column_average, "aligned_depth_vector.txt")
    save_matrix_txt(depth_error,"depth_error_vec.txt")
    save_matrix_txt(aligned_error,"aligned_error.txt")
pipe = rs.pipeline()
cfg = rs.config()
align_to = rs.stream.color
align = rs.align(align_to)

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

pipe.start(cfg)
#output = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MPEG'), 30, (640,480))


while True:
    frame = pipe.wait_for_frames()
    aligned_frames = align.process(frame)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()
    aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imshow('rgb', color_image)
    cv2.imshow('depth', depth_image)
    cv2.imshow('aligned image',aligned_depth_image)
    #output.write(color_image)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        
        save(depth_image, aligned_depth_image)
        break
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
pipe.stop()
#output.release()
