import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
import math
import matplotlib.pyplot as plt
## Problem Setup
# Choose initial state
x0 = np.array([0, 0, 0])

# Choose goal state
xg = np.array([1, 3, math.pi/2])

# Running cost x' Q x + u' R u
# Final cost x'Qf x
Q = 0 * np.identity(3)
R = 0.01 * np.identity(1)
Qf = 100 * np.identity(3)

# Specify u(t) at N knotpoints
N = 1000

# Trajectory duration
T = 5

# Stopping criteria
max_iter = 500
min_err = 1e-6

## Problem initialization

# Time discretization
tvec = np.linspace(0, T, N)

# Initialize random guess
u = .01*np.random.randn(N)

# Initialize stopping criteria
err = np.inf
iter = 0

fig = plt.figure()
plt.ion()
plt.show()

# Dynamics
def dynamics(t, x, tvec, uvec):
  u = np.interp(t, tvec, uvec)
  return [math.sin(x[2]), math.cos(x[2]), u];

def costate_dynamics(t, costate, tvec, xvec, xg):
  # H = costate'f(x,u) + g(x,u)
  # d/dt costate = -partial H/partial x
  
  # partial g/partial x = Q*(x - xg)
  x_fun = interp1d(tvec, xvec.T, axis=0)
  x = x_fun(np.clip(t, tvec[0], tvec[-1]))
  dg_dx = Q @ (x - xg)

  # partial f/ partial x
  # f = [sin(x[2]), cos(x[2]), u]
  theta = x[2]
  df_dx = np.array([[0, 0, math.cos(theta)],
                    [0, 0, -math.sin(theta)],
                    [0, 0, 0]]).T
  return -df_dx @ costate - dg_dx

## Solve problem
while err > min_err and iter <= max_iter:
  # Forward pass integration
  dynamics_autonomous = lambda t, x: dynamics(t, x, tvec, u)
  sol = solve_ivp(dynamics_autonomous, [0, T], x0, t_eval=tvec)

  # Backward pass costate integration
  costate_final = Qf @ (sol.y[:,-1] - xg)
  costate_dynamics_autonomous = lambda t, x: costate_dynamics(t, x, sol.t, sol.y, xg)
  sol_costate = solve_ivp(costate_dynamics_autonomous, [T, 0], costate_final, t_eval=np.flip(tvec))

  # Calculate gradient of cost
  dg_du = R[0, 0] * u.T
  df_du = np.array([0, 0, 1])
  costate = np.flip(sol_costate.y, axis=1)
  
  dJ_du = dg_du + df_du.dot(costate)

  err = np.linalg.norm(dJ_du)
  alpha = 5e-5
  u = u - alpha * dJ_du 
  print(err)

  if iter > 0:
    line[0].remove()
  line = plt.plot(sol.y[0, :], sol.y[1, :])
  plt.draw()
  plt.pause(0.001)

  iter = iter + 1
input()