import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from mpl_toolkits import mplot3d
import matplotlib.animation as animation
from IPython import get_ipython


from cartpole import Cartpole


def plot_cartpole_trajectory(t, x, n_frames):
  theta = x[1, :]
  x = x[0, :]

  px = x + L*np.sin(theta);
  py = - L*np.cos(theta);

  # cartpole geometry parameters
  h = .2;
  w = .4;

  x_range = np.array([-2, 2])
  y_range = np.array([-1, 3])

  pennblue = np.array([1,37,110])/256;
  pennred = np.array([149,0,26])/256;

  fig = plt.figure(figsize=(8,6))
  ax = plt.axes()

  def frame(i):
    ax.clear()

    ax.set_xlim(x_range)
    ax.set_ylim(y_range)

    i = int(i / n_frames * t.shape[0])

    ax.plot(4*x_range, [0,  0], 'k', linewidth=3)
    cartpole_base = Rectangle([x[i]-w/2, -h/2], w, h, facecolor=pennblue, edgecolor='k', linewidth=3)
    cartpole_mass = Circle((px[i], py[i]), 0.02, facecolor=pennred, edgecolor=pennred, linewidth=3)
    ax.add_patch(cartpole_base)
    ax.add_patch(cartpole_mass)
    ax.plot([x[i], x[i] + L*sin(theta[i])], [0, -L*cos(theta[i])], 'k', linewidth=3);
    plot = ax.plot(px[:i], py[:i], 'g', linewidth=3);

    ax.set_xlabel('q1 (m)')

    if not ('google.colab' in str(get_ipython())):
      plt.draw()
      plt.pause(t[-1]/n_frames)
    return plot

  if not ('google.colab' in str(get_ipython())):
    plt.ion()
    plt.show()
    for i in range(n_frames):
      frame(i)

  anim = animation.FuncAnimation(fig, frame, frames=n_frames, blit=False, repeat=False)
  plt.close()
  return anim, fig

def simulate_cartpole(cartpole, x0, tf, plotting=False):
  global g, mc, mp, L

  n_frames = 30

  g = 9.81
  mc = 1
  mp = 1
  L = 1

  def f(t, x):
    M = np.array([[mc + mp, mp*L*cos(x[1])],
                  [mp*L*cos(x[1]), mp*L**2]])
    C = np.array([-mp*L*sin(x[1])*x[3]**2,
                  mp*g*L*sin(x[1])])
    B = np.array([1, 0])

    u = cartpole.compute_efforts(t, x)

    x_dot = np.hstack((x[-2:],
                      np.linalg.solve(M, B * u  - C)))
    return x_dot


  sol = solve_ivp(f, (0, tf), x0, max_step=1e-3)
  if plotting:
    anim, fig = plot_cartpole_trajectory(sol.t, sol.y, n_frames)
    return anim, fig
  else:
    return sol.y

if __name__ == '__main__':
  x0 = np.zeros(4)
  x0[1] = pi/6
  tf = 10
  cartpole = Cartpole()
  simulate_cartpole(cartpole, x0, tf, True)