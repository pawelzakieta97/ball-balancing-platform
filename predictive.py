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
AccMax = 2;

num = np.array(np.append(np.zeros(int(delay/Tp)), [0, 0.0003125, 0.0003125]))
den = np.array([1, -2, 1])  #values derived by matlab with zoh
model = Model(num, den)
model2 = Model(num, den)
gpc = GPC(model2, p, d)
gpc.fi*=100
gpc.updateK()

pid = PID(Tp, 6, 0.02, 4, 0.3)
Yz = 0.1*np.ones(500)
servos = Servos()
fi = np.array([])
for i in range(0,400):
    uPID = max(min((pid.update(model.y[-1], 0.1))*1,AccMax),-AccMax)
    model.update(uPID)
    #uGPC = max(min((model2.u[-1]+gpc.getSteering(Yz)[0])*1,AccMax),-AccMax)
    change = gpc.getSteering(Yz)
    uGPC = model2.u[-1] + change#[0]#gpc.getSteering(Yz)[0]
    model2.update(uGPC)
    fi = np.append(fi, [uGPC])

#print(fi)
#plt.step(range(0,model.y.size-model.yStart), model.y[model.yStart:], where = 'post')
plt.step(range(0,model2.y.size-model2.yStart), 100*model2.y[model2.yStart:], where = 'post')
plt.step(range(0,fi.size), fi, where = 'post')
#plt.step(range(0,model2.u.size-model2.uStart), model2.u[model2.uStart:], where = 'post')
plt.show()

