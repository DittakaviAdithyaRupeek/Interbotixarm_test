import cv2
import math
import datetime
import numpy as np
import pyrealsense2 as rs

# some links from which code is taken in bits and pieces
# https://github.com/IntelRealSense/librealsense/issues/6749 -> How to measure the distance from camera to a pixel on the camera feed?
# https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples -> librealsense python examples
# https://github.com/IntelRealSense/librealsense/issues/4934#issuecomment-537705225 -> Get aligned color and depth with python
# https://github.com/IntelRealSense/librealsense/issues/1904 -> transforming pixel from a depth image to world coordinates

class RealsenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.align = rs.align(rs.stream.depth)
        self.depth_frame = None
        self.color_frame = None

    def start_rs_pipeline(self):
        self.pipeline.start(self.config)

    def stop_rs_pipeline(self):
        self.pipeline.stop()

    def get_frames(self):
        # This call waits until a new coherent set of frames is available on a device
        frames = self.pipeline.wait_for_frames()

        # Aligning color frame to depth frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Save the last grabbed frames
        self.depth_frame = depth_frame
        self.color_frame = color_frame

        return depth_frame, color_frame

    def get_3d_from_pixel(self, x, y):
        if self.depth_frame and self.color_frame:
            color_intrin = self.color_frame.profile.as_video_stream_profile().intrinsics
            # Use pixel value of  depth-aligned color image to get 3D axes
            depth = self.depth_frame.get_distance(x, y)
            dx, dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x, y], depth)

            return dx, dy, dz
        else:
            raise ValueError("Depth and Color Frames not available")

    def get_distance_to_pixel(self, x, y):
        dx, dy, dz = self.get_3d_from_pixel(x, y)
        distance = math.sqrt((dx ** 2) + (dy ** 2) + (dz ** 2))

        return distance

    def click_image(self, depth=True, rgb=True):
        _depth, _rgb = self.get_frames()
        # click and save a depth image
        if depth:
            depth_image = np.asanyarray(_depth.get_data())
            ret = cv2.imwrite('depth_' + str(datetime.datetime.now()) + '.jpg', depth_image)
            if ret:
                print("Clicked and saved a Depth image")
        # click and save an RGB image
        if rgb:
            rgb_image = np.asanyarray(_rgb.get_data())
            ret = cv2.imwrite('rgb_' + str(datetime.datetime.now()) + '.jpg', rgb_image)
            if ret:
                print("Clicked and saved a RGB image")
        

