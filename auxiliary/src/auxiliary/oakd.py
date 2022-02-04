import os
import cv2
import datetime
import numpy as np
import depthai as dai

class OakdCamera:
    def __init__(self, height=1080, width=1920):
        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define source and output
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.xoutRgb.setStreamName("rgb")

        # Properties
        self.camRgb.setPreviewSize(width, height)
        self.camRgb.setInterleaved(False)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        # Linking
        self.camRgb.preview.link(self.xoutRgb.input)

    def click_image(self):
        with dai.Device(self.pipeline) as device:
            # Output queue will be used to get the rgb frames from the output defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
            ret = cv2.imwrite('rgb_' + str(datetime.datetime.now()) + '.jpg', inRgb.getCvFrame())
            if ret:
                print("Clicked and saved a RGB image")