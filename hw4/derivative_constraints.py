import numpy as np
from math import factorial
# from scipy.special import factorial
# Element-wise factorial for arrays

def Ab_i2(i, k, n, d, dt_i):
  '''
  Ab_i1(i, n, d, dt_i, w_i, w_ip1) computes the linear equality constraint
  constants to require the kth derivatives of the ith and (i+1)th
  polynomials to equal where they meet.
  Parameters:
     i - index of the polynomial.
     k - order of derivative being taken.
     n - total number of polynomials.
     d - the number of terms in each polynomial.
     dt_i - Delta t_i, duration of the ith polynomial.
  
     @output A_i1 - A matrix from linear equality constraint A_i1 v = b_i1
     @output b_i1 - b vector from linear equality constraint A_i1 v = b_i1
  '''

  A_i2 = np.zeros((2, 2*d*n))
  b_i2 = np.zeros((2, 1))

  # TODO: Fill in values for A_i2 and b_i2
  
  return A_i2, b_i2