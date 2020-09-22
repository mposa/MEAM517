
import numpy as np
from math import sin, cos, pi

from pydrake.systems.controllers import LinearQuadraticRegulator



class Cartpole(object):
  '''

  '''
  def __init__(self):
    # k_p and k_d gains for swing up controller
    
    # TODO: choose appropriate k_p and k_d gains
    self.kp = 0
    self.kd = 0
    self.g = 9.81

    # mass of the pole and mass of the part
    self.mc = 1
    self.mp = 1
    self.L = 1

    # Computes the lqr gains for the final stabilizing controller
    A = np.array([[0,                  0,                                1, 0],
                  [0,                  0,                                0, 1],
                  [0, self.g * (self.mc / self.mp),                      1, 0],
                  [0, self.g * (self.mc + self.mp) / (self.L * self.mc), 0, 0]])
    B = np.array([0, 0, 1/self.mc, 1/(self.L * self.mc)]).T
    Q = np.eye((4))
    Q[3, 3] = 10
    R = np.array([1])
    N = np.zeros(4)

    self.K, self.s = LinearQuadraticRegulator(A, B, Q, R, N) 
    self.x_des = np.array([0, pi, 0, 0])


  def compute_efforts(self, t, x):
    q = x[:2]
    qdot = x[-2:]

    e_tilde = 0.5 * qdot[1] ** 2 - self.g * cos(q[1]) - self.g
    angular_distance  = x[3]**2 + (x[1]-pi)**2

    if np.abs(e_tilde) < 1 and angular_distance < 1:
      return self.compute_lqr_input(x)
    else:
      return self.compute_energy_shaping_input(t, x)


  def compute_energy_shaping_input(self, t, x):
    '''
    Computes the energy shaping inputs to stabilize the cartpole
    '''
    q = x[:2]
    qdot = x[-2:]
    u = 0
    q1_ddot_des = 0
    q1_ddot_pd_term = - self.kd * qdot[0] - self.kp * q[0]


    # TODO: compute q1_ddot_des 


    # Add pd terms to q1_ddot_des
    q1_ddot_des = q1_ddot_des + q1_ddot_pd_term


    # TODO: compute the optimal input u according to q1_ddot_des and partial feedback linearization

    return u

  def compute_lqr_input(self, x):
    '''
    Stabilizes the cartpole at the final location using lqr
    '''
    return -self.K @ (x - self.x_des)


