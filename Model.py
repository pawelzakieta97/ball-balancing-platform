import numpy as np

class Model:
    def __init__(self, num, den, memory = 1000):
        self.num = num
        self.den = den
        self.y = np.zeros(den.size)
        self.yStart = den.size
        self.u = np.zeros(num.size)
        self.uStart = num.size
        self.memory = memory
        self.step_response = np.array([])
        self.zeros = 8
    def update(self, u_value):
        self.u = np.append(self.u, [u_value])
        steering= np.dot(self.num, self.u[-1:-self.num.size-1:-1])
        output= np.dot(self.den[1:], self.y[-1:-self.den.size:-1])
        self.y = np.append(self.y, [(steering-output)/self.den[0]])
        if self.y.size>2*self.memory:
            self.y = self.y[self.memory:]
        if self.u.size>2*self.memory:
            self.u = self.u[self.memory:]

    def freeTrajectory(self, p):
        if self.step_response.size == 0:
            self.step_response = self.step(200)
            self.zeros = np.count_nonzero(self.step_response == 0)
        model_copy = Model(self.num, self.den)
        model_copy.y = np.array(self.y)
        model_copy.u = np.array(self.u)
        for i in range(0,p):
            model_copy.update(model_copy.u[-1])
        return model_copy.y[self.y.size:]

    def freeTrajectoryFast(self, p):
        if self.step_response.size == 0:
            self.step_response = self.step(200)
            self.zeros = np.count_nonzero(self.step_response == 0)

        model_copy = Model(self.num, self.den)
        model_copy.y = np.array(self.y)
        model_copy.u = np.array(self.u)
        for i in range(0, p):

            if (i == model_copy.num.size):
                prev = model_copy.y[-1]
                pprev = model_copy.y[-2]
                d = prev - pprev
                steering = model_copy.u[-1]
                # I have absolutely no fucking clue why its 1613 but it works
                gain = np.sum(model_copy.num)*1613
                vel = np.fromfunction(lambda j,k: prev+d*(k+1), (1,p-i), dtype=float)
                acc = steering * gain * self.step_response[self.zeros:p - i+self.zeros]
                break

            model_copy.update(model_copy.u[-1])
        return np.append(model_copy.y[self.y.size:], vel+acc)

    def step(self,p):
        model_copy = Model(self.num, self.den)
        for i in range(0,p):
            model_copy.update(1)
        return model_copy.y[-p:]

    
