import io
import time
import picamera
from Servos import Servos
from ImageProcessing import *
from picamera.array import PiRGBArray
from send import *
from Controller import Controller
import numpy as np
import time


'''
This script stabilizes ball and then forces some steering signal to measure system's response.
Results are stored in a file
'''

resY = 360
resX = 480

# PiCamera is a class provided by raspberry, we just have to set it up
with picamera.PiCamera() as camera:
    # selecting optimal settings for the camera
    camera.resolution = (resX, resY)
    camera.framerate = 40
    # shutter speed is the exposure time in us, so this is 3ms. When specified, this value stays constant
    # and doesnt adapt to the lighting conditions but we are sure the delay will be constant
    camera.shutter_speed = 3000
    camera.iso = 800
    # in short, we dont really care about resolution, the only important thing is framerate and camera angle
    # for some reason with framerates higher than 40 the field of view dramatically decreased (camera "zoomed in")

    # apparently the camera needs some time to "worm up"
    time.sleep(2)

    #  controller object is responsible for calculating the appropriate steering signals and executing them
    c = Controller(kind="PID_1")

    # ProcessOutput object creates 4 threads that are responsible for continuous image processing.
    # After each frame is processed, the thread calls an update method from controller object given as constructor argument
    output = ProcessOutput(c)
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(10)#time to stabilize the ball in the middle


    # measuring time at which steering is disabled
    begin = int(round(time.time() * 1000))

    c.steering_enable = False # disabling steering
    s = Servos()
    s.applySteering(0.1, 0) # forcing steering signal that is supposed to result in 0.1 m/s^2 acceleration in x axis
    camera.wait_recording(2)
    output.done = True # setting this field to True results in killing all threads that were processing the image

    # controller object stores all the info. samples dictionary contains 'x', 'y' and 't' numpy arrays
    samples = c.samples
    f = open("acceleration_x.txt", "w+")
    f.write("start: "+begin+"\n")
    for i in range(0, samples['x'].size):
        f.write(samples['t'][i]+"\t"+samples['x'][i]+"\n")

    camera.stop_recording()
