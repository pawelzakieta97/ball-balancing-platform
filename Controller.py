import numpy as np
import time
from numpy.linalg import inv
from PID import PID
from GPC import GPC
from Servos import Servos
from Model import Model
import matplotlib.pyplot as plt

#matrix used to find parameters of polynomial that approximates values
def generateK(power, T):
    B = np.zeros((T.size, power+1))
    for n in range(0,T.size):
        for p in range(0, power+1):
            B[n,p] = pow(T[n],p)
    return np.dot(inv(np.dot(np.transpose(B),B)),np.transpose(B))

#returns a value of a polynomial of given parameters at point x
def polyValue(params, x):
    s = 0
    for i in range(0, params.size):
        s += params[i] * pow(x,i)
    return s

#calculates parameters of a polynomial that is a derivative of polynomial with parameters given as argument
def polyDer(params):
    return np.array([i*params[i] for i in range(1, params.size)])
    
class Controller():
    def __init__(self, Tp=0.025, samples_num=5, poly=2, prevSamples=3, kind="PID_1"):
        self.Tp = Tp
        self.samples = {"x":np.array([]), "ax":np.array([]), "y":np.array([]), "ay":np.array([]), "t":np.array([])}
        self.samples_num = samples_num  #this many previus samples will be considered while fitting the polynomial
        self.parameters = {"x":np.array([]), "ax":np.array([]), "y":np.array([]), "ay":np.array([])}
        self.poly = poly    #this is the gihest power of the polynomial (on default 2- we are fitting a parabola)
        self.begin = int(round(time.time() * 1000)) #current time
        #virtual samples are derived from the polynomial
        self.virtualSamples = {"x": np.array(prevSamples*[[0]]), "y": np.array(prevSamples*[[0]])}
        self.kind = kind    #type of control system- PID_0, PID_1 or GPC defined later

        #creating 2 pid control systems(for x and y axes separately) with default parameters (working best in the simulation)
        self.pid = {"x": PID(Tp, 6, 0.02, 4, 0.3), "y": PID(Tp, 6, 0.02, 4, 0.3)}
        #delay between ball moving and us actually detecting it
        delay = 0.2

        #numerator and denominator of transfer function (capture the behaviour of the model and is used in GPC)
        num = np.array(np.append(np.zeros(int(delay/self.Tp)), [0, 0.0003125, 0.0003125]))
        den = np.array([1, -2, 1])  #values derived by matlab with zoh
        modelx = Model(num, den)
        modely = Model(num, den)
        #creating 2 GPC control systems for x and y. default parameters- we "rate" how well the result would be
        #during 150 samples assuming that it can change steering signal during first 10 samples
        self.gpc = {"x": GPC(modelx, 150, 10), "y": GPC(modely, 150, 10)}
        self.gpc["x"].fi *= 50
        self.gpc["y"].fi *= 50
        self.gpc["x"].updateK()
        self.gpc["y"].updateK()

        #this object delivers methods that take desired acceleration as argument and send aproprieate signals to STM
        self.servos = Servos()

        #this array stores the desired position of the ball in x-y coordinates in the following 150 samples
        self.setPoint = {"x": np.array(150*[0.1]), "y": np.array(150*[0.1])}
        self.steering_enable = True

    #clears memory and updates "begin" value, leaves
    def clear(self, leave=20):
        now = int(round(time.time() * 1000)) - self.begin
        shift = now - self.begin
        self.begin = now
        self.samples['t'] = self.samples['t'][-leave:]
        for i in range(0, self.samples['t']):
            self.samples['t'][i] -= shift/1000
        self.samples['x'] = self.samples['x'][-leave:]
        self.samples['y'] = self.samples['y'][-leave:]

    #this method is called every time new frame has been processed by ImageProcessing class object (58th line).
    #Agruments:
    #   position- x and y coordinates of the ball
    #   timeOverride - if not specified, the controller object will assume that the ball's position is accurate
    #                   at the time of calling the method (now). Otherwise it is overridden- was useful for debugging
    def update(self, position, timeOverride=-10):
        now = timeOverride

        #if timeOverride not specified, timestamp of the sample is the current time
        if timeOverride == -10:
            now = int(round(time.time() * 1000))-self.begin

        #adding new sample to the array - x,y coordinates and it's time in seconds
        self.samples["x"] = np.append(self.samples["x"], position[0])
        self.samples["y"] = np.append(self.samples["y"], position[1])
        self.samples["t"] = np.append(self.samples["t"], float(now/1000))

        #these values will are the desired acceleration of the ball in x and y axes
        ax = 0
        ay = 0

        #Here the desired accelerations are calculated, but only if the number of samples is large enough
        if self.samples["x"].size > self.gpc["x"].model.num.size:

            #matrix K is used to calculate the parameters of polynomial approximation of x and y coordinates over time
            K = generateK(self.poly, self.samples["t"][-self.samples_num:])
            #calculating the parameters
            self.parameters["x"] = np.dot(K, self.samples["x"][-self.samples_num:])
            self.parameters["y"] = np.dot(K, self.samples["y"][-self.samples_num:])

            #virtual samples are values calculated using the approximated polynomial
            self.virtualSamples["x"] = np.array([polyValue(self.parameters["x"], float(now)/1000-(self.gpc["x"].model.den.size-i-1)*self.Tp) for i in range(0, self.gpc["x"].model.den.size)])
            self.virtualSamples["y"] = np.array([polyValue(self.parameters["y"], float(now)/1000-(self.gpc["y"].model.den.size-i-1)*self.Tp) for i in range(0, self.gpc["y"].model.den.size)])

            #first type of control system- most basic PID (calclating derivative by difference between 2 samples)
            if self.kind == "PID_0":
                ax = self.pid["x"].update(self.virtualSamples["x"][-1], self.setPoint["x"][0])
                ay = self.pid["y"].update(self.virtualSamples["y"][-1], self.setPoint["y"][0])

            #second type of control system- the derivative is overriden by one calculated using the approximated polynomial
            if self.kind == "PID_1":
                derParams = polyDer(self.parameters["x"])
                derValue = polyValue(derParams, float(now)/1000)

                der2Value = polyValue(polyDer(derParams), now/1000)
                ax = self.pid["x"].update(self.virtualSamples["x"][-1], self.setPoint["x"][0], derOverride=derValue, der2Override=der2Value)
                
                derParams = polyDer(self.parameters["y"])
                derValue = polyValue(derParams, now/1000)
                der2Value = polyValue(polyDer(derParams), now/1000)
                ay = self.pid["y"].update(self.virtualSamples["y"][-1], self.setPoint["y"][0], derValue, der2Value)

            #third type of control system- predictive algorithm. Given the accurate mathematical model it is able
            #to generate a tajectory of steering signal values such that it minimizes a cost function.
            #Cost function is a sum of errors (difference between set point and actual position) squared
            #plus sum of squared changes of steering signals. The gain of the firs sum is given by parameter "fi"-
            #the bigger the parameter the more valuable will be achiving the set point as fast as possible
            if self.kind == "GPC":
                #GPC control system needs updated coordinates and previus steering signal values
                self.gpc["x"].model.y = self.virtualSamples["x"]
                self.gpc["x"].model.u = self.samples["ax"]
                self.gpc["y"].model.y = self.virtualSamples["y"]
                self.gpc["y"].model.u = self.samples["ay"]
                #the output of "getSteering" is the desired change of steering signal
                ax = self.samples["ax"][-1] + self.gpc["x"].getSteering(self.setPoint["x"])
                ay = self.samples["ay"][-1] + self.gpc["y"].getSteering(self.setPoint["y"])
        #if values "steering_enable" equals True, the calculated steering is applied to the servos
        if self.steering_enable:
            self.servos.applySteering(ax, ay)
        #Adding the calculated values of steering signal to the arrays
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