import numpy as np
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
    pass

  # Aeq, beq = generate_constraint_matrices(n, d, w, dt)
  # H = generate_H_matrix(n, d, dt)

  solver_id = mp.ChooseBestSolver(prog)
  solver = mp.MakeSolver(solver_id)
  result = solver.Solve(prog, None, None)
  v = result.GetSolution()
  import pdb; pdb.set_trace()

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

def generate_H_matrix(n, d, dt):
  '''
  generate_H_matrix(n, d, dt) calculates the matrix for the minimum
  snap cost.
  Parameters:
    n - total number of polynomials.
    d - number of terms in each polynomial.
    dt - Matrix of time spacings, containing Delta t_i in dt[i].
  Outputs:
    H - matrix such that the snap squared integral is equal to
          v^T H v
  '''
  H = np.zeros((2*n*d, 2*n*d))
  # TODO: construct H matrix from H_i1

  # Begin solution code here
  for i in range(n):
    H += 2*H_i1(i, n, d, dt[i]);
  # End solution code here

  return H

def generate_constraint_matrices(n, d, w, dt):
  '''
generate_constraint_matrices(n, d, w, dt) calculates the polynomial coefficients for the minimum
snap trajectory intersecting waypoints w.
  Parameters:
    n - total number of polynomials.
    d - number of terms in each polynomial.
    w - Matrix of waypoints, containing w_i in w[i].
    dt - Matrix of time spacings, containing Delta t_i in dt[i].
  Outputs:
    Aeq - A matrix from linear equality constraint A_eq v = b_eq
    beq - b vector from linear equality constraint A_eq v = b_eq
  '''

  Aeq = np.zeros((0,2*d*n))
  beq = np.zeros((0,1))

  # TODO: Construct constraint matrix and vector Aeq and beq from Ab_i1 and Ab_i2

  # Begin solution code here
  for i in range(n):
      [Aeq_i, beq_i] = Ab_i1(i, n, d, dt[i], w[i], w[i + 1])
      Aeq = np.vstack((Aeq, Aeq_i))
      beq = np.vstack((beq, beq_i))

  for i in range(n - 1):
     for k in range(1, 5):
         [Aeq_i, beq_i] = Ab_i2(i, k, n, d, dt[i]);
         Aeq = np.vstack((Aeq, Aeq_i))
         beq = np.vstack((beq, beq_i))

  # End of solution code
  return Aeq, beq
