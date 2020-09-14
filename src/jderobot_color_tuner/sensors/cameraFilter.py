import numpy as np
import threading
from commm.cameraClient import CameraClient
from commm.threadSensor import ThreadSensor
import cv2
from filters.rgbFilter import RgbFilter
from filters.yuvFilter import YuvFilter
from filters.hsvFilter import HsvFilter

RGB = 'RGB'
HSV = 'HSV'
YUV = 'YUV'
ORIG = 'Orig'


class CameraFilter:

    def __init__(self, camera):
        self.lock = threading.Lock()
        self.client = camera

        img = self.client.getImage()

        self.height= img.height
        self.width = img.width


        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self, self.kill_event)
        self.thread.daemon = True

        rgbfilter = RgbFilter()
        hsvfilter =  HsvFilter()
        yuvfilter = YuvFilter()

        self.filters = {RGB:rgbfilter, YUV:yuvfilter, HSV: hsvfilter}


        if self.client.hasproxy():
             
            trackImage = np.zeros((self.height, self.width,3), np.uint8)
            trackImage.shape = self.height, self.width, 3

            self.images = {ORIG: trackImage, RGB: trackImage, HSV: trackImage, YUV: trackImage} 

            self.kill_event.clear()
            self.thread.start()


    # if client is stopped you can not start again, Threading.Thread raised error
    def stop(self):
        self.kill_event.set()
        self.client.stop()
    
    
    def getImage(self):
        return getOrigImage()

    
    def getFilteredImage(self, filt):
        if self.client.hasproxy():
            img = np.zeros((self.height, self.width,3), np.uint8)
            self.lock.acquire()
            img = self.images[filt]
            img.shape = self.images[filt].shape
            self.lock.release()
            return img
        return None

    def getOrigImage(self):
        return self.getFilteredImage(ORIG)

    def getHSVImage (self):
        return self.getFilteredImage(HSV)

    def getRGBImage (self):
        return self.getFilteredImage(RGB)

    def getYUVImage (self):
        return self.getFilteredImage(YUV)


    def update(self):
        img = self.client.getImage().data
        rgb = self.getFilter(RGB).apply(img)
        hsv = self.getFilter(HSV).apply(img)
        yuv = self.getFilter(YUV).apply(img)

        self.lock.acquire()
        self.images[ORIG] = img
        self.images[RGB] = rgb
        self.images[HSV] = hsv
        self.images[YUV] = yuv
        self.lock.release()



    def getFilter (self, name):
        return self.filters[name]
