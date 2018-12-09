import numpy as np
from numpy.linalg import inv
from Model import Model
import time

class GPC:
    def __init__(self, model, p, d):
        self.p = p
        self.d = d
        self.fi = np.eye(p)
        self.alpha = np.eye(d+1)
        self.model = model
        self.K = np.array([])

    #K is a model-specific matrix that is used to calculate optimal steering
    def updateK(self):
        S = self.getS()
        St = np.transpose(S)
        inside = np.dot(np.dot(St, self.fi), S) + self.alpha
        self.K = np.dot(inv(inside), np.dot(St, self.fi))

    #S is a matrix that stores the impact of steering signal change over time
    def getS(self):
        S = np.zeros((self.p, self.d+1))
        response = self.model.step(self.p)
        S[0,0] = response[0]
        for i in range(1, self.p):
            if i-self.d<=0:
                S[i,0:min(i, self.d)+1] = response[i::-1]
            else:
                S[i,0:min(i, self.d)+1] = response[i:i-self.d-1: -1]
        return S
    def getSteering(self, Yz):
        #begin = (int(round(time.time() * 1000)))
        Y0 = self.model.freeTrajectoryFast(self.p)
        #Y0 = np.ones(self.p)
        #print(int(round(time.time() * 1000))-begin)
        return np.dot(self.K[0],Yz[0:Y0.size]-Y0)
        
