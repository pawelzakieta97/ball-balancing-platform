import io
import time
import threading
import picamera
import cv2
from picamera.array import PiRGBArray
from Controller import Controller

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
                    data = np.fromstring(self.stream.getvalue(), dtype = np.uint8)
                    image = cv2.imdecode(data, 1)
                    image = image[0:self.resY-1, int((self.resX-self.resY)/2):int((self.resX-self.resY)/2)+self.resY]
                    image = cv2.resize(image,(200,200))
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                    lower_green = np.array([40,150,10])
                    upper_green = np.array([90,255,255])

                    lower_blue = np.array([90,150,10])
                    upper_blue = np.array([135, 255,255])
                    
                    mask = cv2.inRange(hsv, lower_blue, upper_blue)
                    denoising = 3
                    mask = cv2.erode(mask, None, iterations = denoising)
                    mask = cv2.dilate(mask, None, iterations = 2*denoising)
                    mask = cv2.erode(mask, None, iterations = denoising)
                    M = cv2.moments(mask)
                    center = [0,0]
                    if M["m00"] != 0:
                        center = [float(M["m10"])/M["m00"], float(M["m01"])/M["m00"]]
                        self.owner.ball_position = center
                        #self.owner.newPosRdy = True
                        self.owner.controller.update(center)
                    
                    if self.owner.frame % 100 == 0:
                        print("processing time in millis: "+str(self.owner.getCurrentTime()-self.owner.processing_begin))
                        name = "image"+str(self.owner.frame)+".jpg"
                        namemask = "image"+str(self.owner.frame)+"_mask.jpg"
                        cv2.circle(image, (int(center[0]),int(center[1])), 10, (0,0,255), 2)
                        cv2.imwrite(name, image)
                        cv2.imwrite(namemask, mask)
                finally:
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
        self.ball_position = [0,0]
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
            #if self.frame == 500:
                #self.done = True
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

