import pyrealsense2 as rs # need to install pyrealsense2 library
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Calculate distance in meters
        depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
        distances = depth_image * depth_scale

        # Display distance at center of the image
        center_x = depth_frame.get_width() // 2
        center_y = depth_frame.get_height() // 2
        distance_at_center = distances[center_y, center_x]
        print(distance_at_center)
        

        #center_x will be the known distance
        
        
        # print("Distance at center: {:.2f} meters".format(distance_at_center))

finally:
    # Stop streaming
    pipeline.stop()

