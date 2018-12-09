import io
import time
import picamera
from ImageProcessing import *
from picamera.array import PiRGBArray
from send import *
from Controller import Controller
import numpy as np
import time

resY = 360
resX = 480


with picamera.PiCamera() as camera:
    camera.resolution = (resX, resY)
    camera.framerate = 40
    camera.shutter_speed = 3000
    camera.iso = 800
    time.sleep(2)
    c = Controller()
    output = ProcessOutput(c)
    camera.start_recording(output, format='mjpeg')
    while not output.done:
        if output.newPosRdy:
            print(output.ball_position)
            output.newPosRdy = False
        camera.wait_recording(1)
    camera.stop_recording()
