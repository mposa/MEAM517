import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from quadrotor import Quadrotor
from trajectories import *
import matplotlib.pyplot as plt

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
