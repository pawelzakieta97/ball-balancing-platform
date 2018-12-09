import matplotlib.pyplot as plt
from Model import Model
from GPC import GPC
from PID import PID
import numpy as np
from Servos import Servos

delay = 0.2
p = 130
d = 10
Tp = 0.025
AccMax = 2


num = np.array(np.append(np.zeros(int(delay/Tp)), [0, 0.0003125, 0.0003125]))
den = np.array([1, -2, 1])  #values derived by matlab with zoh
model2 = Model(num, den)
for i in range(0,10):
    model2.update(i)

#model2.update(0)
step1 = model2.freeTrajectory(130)
step2 = model2.freeTrajectoryFast(130)
gpc = GPC(model2, p, d)
gpc.fi*=100
gpc.updateK()

Yz = 0.1*np.ones(500)
servos = Servos()
fi = np.array([])
# for i in range(0,400):
#     change = gpc.getSteering(Yz)
#     uGPC = model2.u[-1] + change
#     model2.update(uGPC)
#     fi = np.append(fi, [uGPC])

#plt.step(range(0,model2.y.size-model2.yStart), 100*model2.y[model2.yStart:], where = 'post')
plt.step(range(0,step1.size), step1, where = 'post')
plt.step(range(0,step2.size), step2, where = 'post')
plt.show()

