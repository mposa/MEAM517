import numpy as np
from math import factorial

def H_i1(i, n, d, dt_i):
  '''
  H_i1(i, n, d, dt_i) computes the integral of snap squared over
  the ith polynomial.
  Parameters:
    i - index of the polynomial.
    n - total number of polynomials.
    d - number of terms in each polynomial.
    dt_i - Delta t_i, duration of the ith polynomial.
  Outputs:
    H_i - matrix such that the snap squared integral is equal to
          v^T H_i v
  '''
  # Note H_i is the same size as H. However for each i, you will only be
  # modifying a small portion of the H matrix
  H_i = np.zeros((2*d*n, 2*d*n))

  # TODO: Fill in values for H_i

  return H_i

