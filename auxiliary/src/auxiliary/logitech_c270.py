import os
import cv2
import datetime

class LogitechC270Camera:
    def __init__(self, device=0):
        self.camera = cv2.VideoCapture(device)

    def camera_info(self):
        test = self.camera.get(cv2.cv2.CV_CAP_PROP_POS_MSEC)
        ratio = self.camera.get(cv2.cv2.CV_CAP_PROP_POS_AVI_RATIO)
        frame_rate = self.camera.get(cv2.cv2.CV_CAP_PROP_FPS)
        width = self.camera.get(cv2.cv2.CV_CAP_PROP_FRAME_WIDTH)
        height = self.camera.get(cv2.cv2.CV_CAP_PROP_FRAME_HEIGHT)
        brightness = self.camera.get(cv2.cv2.CV_CAP_PROP_BRIGHTNESS)
        contrast = self.camera.get(cv2.cv2.CV_CAP_PROP_CONTRAST)
        saturation = self.camera.get(cv2.cv2.CV_CAP_PROP_SATURATION)
        hue = self.camera.get(cv2.cv2.CV_CAP_PROP_HUE)
        gain = self.camera.get(cv2.cv2.CV_CAP_PROP_GAIN)
        exposure = self.camera.get(cv2.cv2.CV_CAP_PROP_EXPOSURE)
        print("Test: ", test)
        print("Ratio: ", ratio)
        print("Frame Rate: ", frame_rate)
        print("Height: ", height)
        print("Width: ", width)
        print("Brightness: ", brightness)
        print("Contrast: ", contrast)
        print("Saturation: ", saturation)
        print("Hue: ", hue)
        print("Gain: ", gain)
        print("Exposure: ", exposure)

    def get_image(self):
        ret, img = self.camera.read()
        if ret:
            return img
        else:
            return None

    def capture_image(self, save=True, name=None, show=False, disp_time=500):
        ret, img = self.camera.read()
        if ret:
            if show:
                cv2.imshow('Window', img)
                cv2.waitKey(disp_time)
            if save:
                if name:
                    suc = cv2.imwrite(name, img)
                else:
                    suc = cv2.imwrite(str(datetime.datetime.now())+'.png', img)
                if suc:
                    print("Image written to %s" % (os.getcwd()))
        return ret

    
        
    
