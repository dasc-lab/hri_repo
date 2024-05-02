import socket
import struct
import numpy as np
import pickle
from pyrealsense2 import pyrealsense2 as rs

def send_frame(conn, frame):
    # Serialize the frame using pickle
    serialized_frame = pickle.dumps(frame)
    # Send the length of the serialized frame
    conn.sendall(struct.pack("L", len(serialized_frame)))
    # Send the serialized frame
    conn.sendall(serialized_frame)

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 5000))
    print("listen")
    server_socket.listen(1)

    connection, addr = server_socket.accept()
    print("connection")

    pipeline = rs.pipeline()
    config = rs.config()

    # Configure the RealSense pipeline
    # Add necessary stream configurations based on your requirements
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    try:
        while True:
            # Wait for a coherent pair of frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # Convert the color frame to a numpy array
            frame_data = np.asanyarray(color_frame.get_data())

            # Send the frame to the client
            send_frame(connection, frame_data)
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
