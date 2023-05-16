from math import *
from scipy.integrate import odeint
from models.agent import *

class RobotSingleIntegrator2D(Agent):

    def __init__(self, size, velocity, k):
        self.size = size
        self.velocity = velocity
        self.k = k

    def model(self, q, t, u): 
        return u

    def controller(self, q, qref, uref):
        u = uref + self.k.dot(qref-q)
        return u 

    def bloating(self, n):
        return 0
        k1, k2 = self.k
        if n != 0:
            return sqrt(4 * n / k2)
        else:
            return sqrt(4 / k2)

    def run_model(self, q0, t, qref, uref):
        q = [q0]
        u0 = [0, 0]
        for i in range(0,len(t)):
            t_step = [t[i-1], t[i]]

            q1 = odeint(self.model, q0, t_step, args = (u0,)) 
                                                            
            q0 = q1[1]
            q.append(q0)
            u0 = self.controller(q0, qref[i], uref[i])
        return q