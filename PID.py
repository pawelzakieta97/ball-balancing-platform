import numpy as np
from numpy.linalg import inv
from Model import Model

class PID:
    def __init__(self, Tp, P, I=0, D=0, DD=0, maxI = 1):
        self.Tp = Tp
        self.P = P
        self.I = I
        self.D = D
        self.DD = DD
        self.integral = 0
        self.previusError = 0
        self.previusDer = 0
        self.maxI = maxI

    def update(self, currentValue, setValue, derOverride = 696969, der2Override = 696969):
        Error = setValue - currentValue
        self.integral += Error*self.Tp
        if(self.integral>self.maxI):
            self.integral = self.maxI
        if(self.integral<-self.maxI):
            self.integral = -self.maxI
        derivative = (Error-self.previusError)/self.Tp
        derivative2 = (derivative-self.previusDer)/self.Tp
        self.previusError = Error
        self.previusDer = derivative
        
        #you can override derivatives with pre processed values
        if derOverride!=696969:
            derivative = -derOverride
        if der2Override!=696969:
            derivative2 = -der2Override

        #print (derivative)
        return self.P*Error + self.I*self.integral + self.D*derivative + self.DD*derivative2
        
