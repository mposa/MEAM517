import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from quadrotor import Quadrotor
from trajectories import *
import matplotlib.pyplot as plt

# Dynamics for the quadrotor
def _f(x, u):
  g = 9.81
  m = 1
  a = 0.25
  I = 0.0625

  theta = x[2]
  ydot = x[3]
  zdot = x[4]
  thetadot = x[5]
  u0 = u[0]
  u1 = u[1]

  xdot = np.array([ydot,
                   zdot,
                   thetadot,
                   -sin(theta) * (u0 + u1) / m,
                   -g + cos(theta) * (u0 + u1) / m,
                   a * (u0 - u1) / I])

  return xdot

def simulate_quadrotor(x0, tf, quadrotor):
  # Simulates a stabilized maneuver on the 2D quadrotor
  # system, with an initial value of x0
  t0 = 0.0
  n_points = 1000
  
  dt = 1e-3

  x = [x0]
  u = [np.zeros((2,))]
  t = [t0]
  
  while t[-1] < tf:

    current_time = t[-1]
    current_x = x[-1]
    current_u = quadrotor.compute_feedback(current_time, current_x)
    # Autonomous ODE for constant inputs to work with solve_ivp
    def f(t, x):
      return _f(current_x, current_u)
    # Integrate one step
    sol = solve_ivp(f, (0, dt), current_x, first_step=dt)

    # Record time, state, and inputs
    t.append(t[-1] + dt)
    x.append(sol.y[:, -1])
    u.append(current_u)

  x = np.array(x)
  u = np.array(u)
  t = np.array(t)
  return x, u, t

if __name__ == '__main__':
  tf = 2*pi;
  R = np.eye(2);
  Q = np.diag([10, 10, 1, 1, 1, 1]);
  Qf = Q;

  quadrotor = Quadrotor(Q, R, Qf, tf);

  x0 = 0.5 * np.ones((6,)) + x_d(0.0)
  x, u, t = simulate_quadrotor(x0, tf, quadrotor)
  plt.plot(x[:, 0], x[:, 1])

  n_samples = 1000
  t_samples = np.linspace(0.0, tf, n_samples)
  x_des = np.zeros((n_samples, 6))
  for i in range(t_samples.shape[0]):
    x_des[i] = x_d(t_samples[i])
  plt.plot(x_des[:, 0], x_des[:, 1], label='desired trajectory')
  plt.plot(x[:, 0], x[:, 1], label='actual trajectory')
  plt.legend()
  plt.show()