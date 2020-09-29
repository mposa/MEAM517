import numpy as np
from pos_constraints import Ab_i1
from derivative_constraints import Ab_i2
from minsnap_cost import H_i1
from scipy.interpolate import PPoly

from cvxopt import matrix


def minsnap(n, d, w, dt):
  nvars = 2*d*n

  Aeq, beq = generate_constraint_matrices(n, d, w, dt)
  H = generate_H_matrix(n, d, dt)


  H = matrix(H)
  f = matrix(0.0, (nvars, 1))
  G = matrix(0.0, (0, nvars))
  h = matrix(0.0, (0, 1))
  A = matrix(Aeq)
  b = matrix(beq)
  result = qp(H, f, G, h, A, b)

  # Extract v from the QP solution
  v = np.array(result['x'])

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

  return Aeq, beq
