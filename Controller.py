import numpy as np
import time
from numpy.linalg import inv
from PID import PID
from GPC import GPC
from Servos import Servos
from Model import Model
import matplotlib.pyplot as plt

#matrix used to find parameters of polynomial
def generateK(power, T):
    B = np.zeros((T.size, power+1))
    for n in range(0,T.size):
        for p in range(0, power+1):
            B[n,p] = pow(T[n],p)
    return np.dot(inv(np.dot(np.transpose(B),B)),np.transpose(B))

def polyValue(params, x):
    s = 0
    for i in range(0, params.size):
        s += params[i] * pow(x,i)
    return s

def polyDer(params):

    return np.array([i*params[i] for i in range(1, params.size)])
    
class Controller():
    def __init__(self, Tp=0.025, samples_num=5, poly=2, prevSamples=3, kind="PID_1"):
        self.Tp = Tp
        self.samples = {"x":np.array([]), "ax":np.array([]), "y":np.array([]), "ay":np.array([]), "t":np.array([])}
        self.samples_num = samples_num  #this many previus samples will be considered while fitting the polynomial
        self.parameters = {"x":np.array([]), "ax":np.array([]), "y":np.array([]), "ay":np.array([])}
        self.poly = poly
        self.begin = int(round(time.time() * 1000))
        #virtual samples are derived from the polynomial
        self.virtualSamples = {"x": np.array(prevSamples*[[0]]), "y": np.array(prevSamples*[[0]])}
        self.kind = kind
        self.pid = {"x": PID(Tp, 6, 0.02, 4, 0.3), "y": PID(Tp, 6, 0.02, 4, 0.3)}
        delay = 0.2
        num = np.array(np.append(np.zeros(int(delay/self.Tp)), [0, 0.0003125, 0.0003125]))
        den = np.array([1, -2, 1])  #values derived by matlab with zoh
        modelx = Model(num, den)
        modely = Model(num, den)
        self.gpc = {"x": GPC(modelx, 150, 10), "y": GPC(modely, 150, 10)}
        self.gpc["x"].fi *= 50
        self.gpc["y"].fi *= 50
        self.gpc["x"].updateK()
        self.gpc["y"].updateK()
        self.servos = Servos()
        self.setPoint = {"x": np.array(150*[0.1]), "y": np.array(150*[0.1])}
        self.steering_enable = True

    #clears memory and updates "begin" value, leaves
    def clear(self, leave=20):
        now = int(round(time.time() * 1000)) - self.begin
        shift = now - self.begin
        self.begin = now
        self.samples['t'] = self.samples['t'][-leave:]
        for i in range(0,self.samples['t']):
            self.samples['t'][i] -= shift/1000
        self.samples['x'] = self.samples['x'][-leave:]
        self.samples['y'] = self.samples['y'][-leave:]

    def update(self, position, timeOverride=-10):
        now = timeOverride
        if timeOverride == -10:
            now = int(round(time.time() * 1000))-self.begin

        self.samples["x"] = np.append(self.samples["x"], position[0])
        self.samples["y"] = np.append(self.samples["y"], position[1])
        self.samples["t"] = np.append(self.samples["t"], float(now/1000))
        ax = 0
        ay = 0
        if self.samples["x"].size > self.gpc["x"].model.num.size:
            K = generateK(self.poly, self.samples["t"][-self.samples_num:])
            self.parameters["x"] = np.dot(K, self.samples["x"][-self.samples_num:])
            self.parameters["y"] = np.dot(K, self.samples["y"][-self.samples_num:])

            self.virtualSamples["x"] = np.array([polyValue(self.parameters["x"], float(now)/1000-(self.gpc["x"].model.den.size-i-1)*self.Tp) for i in range(0, self.gpc["x"].model.den.size)])
            self.virtualSamples["y"] = np.array([polyValue(self.parameters["y"], float(now)/1000-(self.gpc["y"].model.den.size-i-1)*self.Tp) for i in range(0, self.gpc["y"].model.den.size)])


            if self.kind == "PID_0":
                ax = self.pid["x"].update(self.virtualSamples["x"][-1], self.setPoint["x"][0])
                ay = self.pid["y"].update(self.virtualSamples["y"][-1], self.setPoint["y"][0])


            if self.kind == "PID_1":
                derParams = polyDer(self.parameters["x"])
                derValue = polyValue(derParams, float(now)/1000)

                der2Value = polyValue(polyDer(derParams), now/1000)
                ax = self.pid["x"].update(self.virtualSamples["x"][-1], self.setPoint["x"][0], derOverride=derValue, der2Override=der2Value)
                
                derParams = polyDer(self.parameters["y"])
                derValue = polyValue(derParams, now/1000)
                der2Value = polyValue(polyDer(derParams), now/1000)
                ay = self.pid["y"].update(self.virtualSamples["y"][-1], self.setPoint["y"][0], derValue, der2Value)

            if self.kind == "GPC":

                self.gpc["x"].model.y = self.virtualSamples["x"]
                self.gpc["x"].model.u = self.samples["ax"]
                self.gpc["y"].model.y = self.virtualSamples["y"]
                self.gpc["y"].model.u = self.samples["ay"]

                ax = self.samples["ax"][-1] + self.gpc["x"].getSteering(self.setPoint["x"])
                ay = self.samples["ay"][-1] + self.gpc["y"].getSteering(self.setPoint["y"])

        if self.steering_enable:
            self.servos.applySteering(ax, ay)
        self.samples["ax"] = np.append(self.samples["ax"], [ax])
        self.samples["ay"] = np.append(self.samples["ay"], [ay])
            
    
c = Controller(kind="GPC")
delay = 0.2
Tp = 0.025
num = np.array(np.append(np.zeros(int(delay/Tp)), [0, 0.0003125, 0.0003125]))
den = np.array([1, -2, 1])
modelx = Model(num, den)
modely = Model(num, den)
maxAcc = 2
begin = int(round(time.time() * 1000))
for i in range(0, 200):
    c.update([modelx.y[-1], modely.y[-1]], 25*i)
    modelx.update(max(min(c.samples["ax"][-1],maxAcc),-maxAcc))
    modely.update(max(min(c.samples["ay"][-1],maxAcc),-maxAcc))
print(int(round(time.time() * 1000))-begin)
plt.step(range(0,modelx.y.size), 10*modelx.y, where = 'post')
plt.step(range(0,modelx.u.size), modelx.u, where = 'post')
plt.show()
#print(c.parameters)
#print(c.virtualSamples)

