import numpy as np
from math import factorial
from pos_constraints import Ab_i1
from scipy.interpolate import PPoly

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver

def minsnap(n, d, w, dt):
  # n_coeffs_per_segment = 2*d
  n_dim = 2
  dim_names = ['y', 'z']

  prog = mp.MathematicalProgram()
  # sigma is a (n, d, n_dim) matrix of decision variables
  sigma = np.zeros((n, d, n_dim), dtype="object")
  for i in range(n):
    for j in range(d):
      sigma[i][j] = prog.NewContinuousVariables(n_dim, "sigma_" + str(i) + ',' + str(j)) 

  # Add A_i1 constraints here
  for i in range(n):
    Aeq_i, beq_i = Ab_i1(i, n, d, dt[i], w[i], w[i + 1])
    prog.AddLinearEqualityConstraint(Aeq_i, beq_i, sigma.flatten())


  # TDOO: Add A_i2 constraints here
  # Hint: Use AddLinearEqualityConstraint(expr, value)

  # TODO: Add cost function here

  solver = OsqpSolver()
  result = solver.Solve(prog)
  print(result.get_solution_result())
  v = result.GetSolution()
  
  # Reconstruct the trajectory from the polynomial coefficients
  coeffs_y = v[::2]
  coeffs_z = v[1::2]
  y = np.reshape(coeffs_y, (d, n), order='F')
  z = np.reshape(coeffs_z, (d, n), order='F')
  coeff_matrix = np.stack((np.flip(y, 0), np.flip(z, 0)), axis=-1)  
  t0 = 0
  t = np.hstack((t0, np.cumsum(dt)))
  minsnap_trajectory = PPoly(coeff_matrix, t, extrapolate=False)

  return minsnap_trajectory

if __name__ == '__main__':

  n = 3;
  d = 14;

  w = np.zeros((4, 2))
  dt = np.zeros(3)

  w[0] = np.array([-3,-4])
  w[1] = np.array([ 0, 0])
  w[2] = np.array([ 2, 3])
  w[3] = np.array([ 5, 0])

  dt[0] = 1;
  dt[1] = 1;
  dt[2] = 1;

  minsnap_trajectory = minsnap(n, d, w, dt)

  import matplotlib.pyplot as plt
  t = np.linspace(0, 3, 100)
  fig = plt.figure(figsize=(4,3))
  ax = plt.axes()
  ax.scatter(w[:, 0], w[:, 1], c='b', label='way pts')
  ax.plot(minsnap_trajectory(t)[:,0], minsnap_trajectory(t)[:,1], label='0')
  ax.legend()
  plt.show()