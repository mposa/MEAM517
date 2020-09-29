import numpy as np
from math import factorial
from pos_constraints import Ab_i1
from derivative_constraints import Ab_i2
from minsnap_cost import H_i1
from scipy.interpolate import PPoly

from pydrake.solvers import mathematicalprogram as mp



def minsnap(n, d, w, dt):
  n_coeffs_per_segment = 2*d


  prog = mp.MathematicalProgram()
  sigma = np.zeros((n, n_coeffs_per_segment), dtype="object")
  for i in range(n):
    sigma[i] = prog.NewContinuousVariables(n_coeffs_per_segment, "sigma_" + str(i)) 


  # Add A_i1 constraints here
  for i in range(n):
    Aeq_i, beq_i = Ab_i1(i, n, d, dt[i], w[i], w[i + 1])
    prog.AddLinearEqualityConstraint(Aeq_i, beq_i, sigma.flatten())

  # Add A_i2 constraints here
  for i in range(n - 1):
    for k in range(1, 5):
      constraint_expr = -factorial(k)
      for j in range(k, d):
        constraint_expr += factorial(j) / factorial(j - k) * sigma[i,j] * dt[i] ** (j-k)
    prog.AddLinearEqualityConstraint(constraint_expr, 0)

  solver_id = mp.ChooseBestSolver(prog)
  solver = mp.MakeSolver(solver_id)
  result = solver.Solve(prog, None, None)
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
  import pdb; pdb.set_trace()

  return minsnap_trajectory
