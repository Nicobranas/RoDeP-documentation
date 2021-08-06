from abc import ABC, abstractmethod
import time
import cv2

class ICamera(ABC):
    @abstractmethod
    def grab(self):
        pass
    
class OpenCVCamera(ICamera):
    def __init__(self, width, height, fps):
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        #self.camera.set(cv2.CAP_PROP_FPS, fps)
        # Set the internal buffer size to 1 to avoid accumulating old
        # images.
        #self.camera.set(cv2.CV_CAP_PROP_BUFFERSIZE, 1)
        self.grab_count = 0
        self.start_time = time.time()
        self.time_last_grab = time.time()
        self.__warm_up()
        print("Camera: estimated FPS: %d" % int(self.estimate_framerate()))
        
    def __warm_up(self):
        for i in range(100):
            img = self.grab()
        
    def estimate_framerate(self):
        duration = self.time_last_grab - self.start_time
        return self.grab_count / duration
            
    def grab(self):
        ret, img = self.camera.read()
        self.time_last_grab = time.time()
        self.grab_count += 1
        return img

def make_camera(config=None):
    # We don't use config, yet. Simply return an OpenCVCamera with
    # fixed settings.
    return OpenCVCamera(640, 480, 60)

