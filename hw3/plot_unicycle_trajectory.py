import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import *
from IPython import get_ipython

from matplotlib import rc
rc('animation', html='jshtml')

def plot_unicycle_trajectory(t, x, y_spline, z_spline, n_frame = 10):
  y_d = y_spline(t);
  z_d = z_spline(t);

  x_max = max(np.max(y_d) + 1, np.max(x[:, 0]))
  x_min = min(np.min(y_d) - 1, np.min(x[:, 0]))
  y_max = max(np.max(z_d) + 1, np.max(x[:, 1]), 3.5)
  y_min = min(np.min(z_d) - 1, np.min(x[:, 1]), -3.5)

  frame_idx = [round(x) for x in np.linspace(0, x.shape[0]-1, n_frame).tolist()]
  x_anim = np.zeros((n_frame, 3))
  for i in range(n_frame):
    x_anim[i, :] = x[frame_idx[i], :]

  a = 0.3;
  y = x_anim[:,0]
  z = x_anim[:,1]
  theta = x_anim[:,2]

  fig = plt.figure(figsize=(8,6))
  ax = plt.axes()

  def frame(i):
    ax.clear()

    # Start and end points
    ax.scatter(0, 0, c='b', label='Initial position')
    ax.scatter(10, 0, c='g', label='Target position')

    # Trajectories
    ax.plot(y_d, z_d, label='Desired trajectory')
    ax.plot(x[:frame_idx[i], 0], x[:frame_idx[i], 1], '--', label='Actual trajectory')

    # Obstacle
    a_circle = plt.Circle((5, 0), 3, color='r', label='Obstacle')
    ax.add_artist(a_circle)
    # Unicycle
    plot = ax.plot([y[i] + a*cos(theta[i]), y[i] - a*cos(theta[i])],
                   [z[i] + a*sin(theta[i]), z[i] - a*sin(theta[i])] , 'k', LineWidth=5, label='Unicycle')

    ax.set_xlim(x_min,x_max)
    ax.set_ylim(y_min,y_max)
    ax.set_xlabel('y (m)')
    ax.set_ylabel('z (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper right')

    if not ('google.colab' in str(get_ipython())):
      plt.draw()
      plt.pause(t[-1]/n_frame)

    return plot

  if not ('google.colab' in str(get_ipython())):
    plt.ion()
    plt.show()
    for i in range(n_frame):
      frame(i)

  anim = animation.FuncAnimation(fig, frame, frames=n_frame, blit=False, repeat=False)
  plt.close()

  return anim
