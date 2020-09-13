import numpy as np
from math import *
from grid_world import *

import matplotlib.pyplot as plt

def plot_value_function_and_optimal_policy(world, V, u_opt):
  plt.clf()
  v_plot = plt.imshow(V, interpolation='nearest')
  colorbar = plt.colorbar()
  colorbar.set_label("Value function")
  plt.xlabel("Column")
  plt.ylabel("Row")
  arrow_length = 0.25
  for row in range(world.rows):
    for col in range(world.cols):
      if u_opt[row, col] == 0: #N
        plt.arrow(col, row, 0, -arrow_length, head_width=0.1)
      elif u_opt[row, col] == 1: #E
        plt.arrow(col, row, arrow_length, 0, head_width=0.1)
      elif u_opt[row, col] == 2: #S
        plt.arrow(col, row, 0, arrow_length, head_width=0.1)
      elif u_opt[row, col] == 3: #W
        plt.arrow(col, row, -arrow_length, 0, head_width=0.1)
      else:
        raise ValueError("Invalid action")
  plt.show()

def value_iteration(world, threshold, gamma, plotting=True):
  V = np.zeros((world.rows, world.cols))
  u_opt = np.zeros((world.rows, world.cols))
  grid_x, grid_y = np.meshgrid(np.arange(0, world.rows, 1), 
                               np.arange(0, world.cols, 1))
  delta = 10.0

  fig = plt.figure("Gridworld")
  # TODO: calculate V and the optimal policy u using value iteration

  if plotting:
    plot_value_function_and_optimal_policy(world, V, u_opt)

  return V, u_opt

if __name__=="__main__":
  world = Gridworld() 
  threshold = 0.0001
  gamma = 0.9
  # value_iteration(world, threshold, gamma, False)
  value_iteration(world, threshold, gamma, True)
