import io
import time
import threading
#import picamera
import cv2
#from picamera.array import PiRGBArray

import numpy as np
import time


class ImageProcessor(threading.Thread):
    def __init__(self, owner, resX = 480, resY = 360):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()
        self.resX = resX
        self.resY = resY
        self.resize = 150 # resolution after downscaling
        self.FOV = 50 # camera angle in degrees, NEEDS TO BE CHECKED!!!!!!!!
        self.h = 0.25 # how high the camera is above the platform
        self.d = self.resize/2/np.tan(np.deg2rad(self.FOV/2))   # a helper value. it shows how many pixels away from the camera
                                                    # is the matrix (makes sense on a drawing)

    # this methods converts the position of the ball center in pixel coordinates into platform based coordinates system
    # in meters. These values will be used by the controller
    def pix2meters(self, x, y):
        x -= self.resize/2
        y = -y + self.resize/2
        A = self.owner.controller.alfa
        B = self.owner.controller.beta
        sA = np.sin(A)
        sB = np.sin(B)
        cA = np.cos(A)
        cB = np.cos(B)
        denominator = -x*sB+y*sA*cB+self.d*cA*cB
        x = self.h*(x*cA*cB*cB + y*sA*sB*cA*cB + self.d*sB*cA*cA*cB) / denominator - self.h*sB*cA
        y = self.h*(y*cA*cA*cB - self.d*sA*cA*cB) / denominator + self.h*sA
        return[x, y]
    def run(self):
        # This method runs in a separate thread
        imnum = 0
        kernel = np.ones((5,5),np.uint8)
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    self.owner.frame += 1
                    self.stream.seek(0)
                    # Read the image and do some processing on it

                    #i'm not sure what is going on here, it reads some raw data and converts it to an array i guess
                    data = np.fromstring(self.stream.getvalue(), dtype=np.uint8)
                    image = cv2.imdecode(data, 1)

                    # cropping the image to a square (height stays the same but sides are cut off)
                    image = image[0:self.resY-1, int((self.resX-self.resY)/2):int((self.resX-self.resY)/2)+self.resY]
                    # downsampling the image to resolution resize x resize to speed up the processing
                    image = cv2.resize(image, (self.resize, self.resize))
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)# converting to hsv color space

                    # definition of different colors ranges in hsv
                    lower_green = np.array([40, 150, 10])
                    upper_green = np.array([90, 255, 255])

                    lower_blue = np.array([90, 150, 10])
                    upper_blue = np.array([135, 255, 255])

                    lower_orange = np.array([10, 190, 10])
                    upper_orange = np.array([30, 255, 255])

                    # pixels that are between sepcified color range are white, all the rest is black
                    mask = cv2.inRange(hsv, lower_orange, upper_orange)
                    denoising = 3

                    # https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
                    mask = cv2.erode(mask, None, iterations=denoising)
                    mask = cv2.dilate(mask, None, iterations=2*denoising)
                    mask = cv2.erode(mask, None, iterations=denoising)

                    # M object will be used to find the center of mass of the white pixels
                    M = cv2.moments(mask)
                    # definig default ball position in case no pixel in given range is detected
                    center = [self.resize/2, self.resize/2]
                    if M["m00"] != 0:
                        # calculating center of mass of the white pixels- this will be the center of the ball
                        center = [float(M["m10"])/M["m00"], float(M["m01"])/M["m00"]]
                        #here the calculated position is given to the controller so that it can process it and apply steering
                        self.owner.controller.update(self.pix2meters(center[0], center[1]))

                    # saving image every 100 frames to see if image is processed correctly.
                    # Should be disabled in final version because it takes a lot of time to save an image
                    if self.owner.frame % 100 == 0:
                        print("processing time in millis: "+str(self.owner.getCurrentTime()-self.owner.processing_begin))
                        name = "image"+str(self.owner.frame)+".jpg"
                        namemask = "image"+str(self.owner.frame)+"_mask.jpg"
                        cv2.circle(image, (int(center[0]),int(center[1])), 10, (0,0,255), 2)
                        cv2.imwrite(name, image)
                        cv2.imwrite(namemask, mask)
                finally:
                    # some stream and thread managing stuff
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

class ProcessOutput(object):
    def __init__(self, controller):
        self.done = False
        self.newPosRdy = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.frame = 0
        self.pool = [ImageProcessor(self) for i in range(4)]
        self.processor = None
        self.processing_begin = 0
        self.controller = controller

    def getCurrentTime(self):
        return int(round(time.time() * 1000))

    def write(self, buf):
        self.processing_begin = self.getCurrentTime()
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.frame % 100 == 0:
                print(self.frame)
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    print("frame missed")
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()
