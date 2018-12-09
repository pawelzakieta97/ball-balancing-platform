import numpy as np
from numpy.linalg import inv
from Model import Model
#import serial

class Servos:
    def __init__(self, armLength = 1.8, joint2link = 4.0, minAngle = -60, maxAngle= 60, maxTilt = 30):
        self.r= armLength
        self.d = joint2link
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.maxTilt = maxTilt
        
    def applySteering(self, ax, ay):
        g = 9.81
        ax = min(max(ax,-g),g)
        ay = min(max(ay,-g),g)
        d = self.d
        r = self.r
        alfa = min(max(np.arcsin(-5*ay/3/g), np.deg2rad(-self.maxTilt)),np.deg2rad(self.maxTilt))
        #print(np.rad2deg(alfa))
        beta = min(max(np.arcsin(5*ax/3/g/np.cos(alfa)), np.deg2rad(-self.maxTilt)),np.deg2rad(self.maxTilt))
        #print(d/r*np.sin(alfa))
        fi1 = np.rad2deg(np.arcsin(d/r*np.sin(alfa)))
        fi2 = np.rad2deg(np.arcsin(d/r*np.sin(alfa)*np.sin(beta)))
        fi1 = min(max(fi1, self.minAngle),self.maxAngle)
        fi2 = min(max(fi2, self.minAngle), self.maxAngle)
        self.send(fi1,fi2)
        return (fi1, fi2)
        
    def send(self, a1, a2):
        #ser = serial.Serial('/dev/ttyACM1',115200)
        #ser.write({fi1})
        #ser.write({fi2})
        return 0
