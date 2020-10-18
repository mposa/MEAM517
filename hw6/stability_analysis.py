import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables
from quadrotor import Quadrotor

def add_sos_constraint(prog, x, V, Vdot):
  # TODO: Impose constraint on Vdot
  # Use prog.AddSosConstraint(expression)
  pass

def add_cost(prog, S):
  # TODO: Add cost
  # Use prog.AddLinearCost(expression)
  pass

def compute_region_of_attraction(quadrotor):
  # Initialize an empty optimization program.
  prog = MathematicalProgram()
  # Declare "x" as indeterminates
  x = prog.NewIndeterminates(6)
  # Declare a SOS polynomial for the lyapunov function
  # V = x.T @ S @ x
  # where "S" is the decision variables in the optimization problem.
  m = OddDegreeMonomialBasis(Variables(x), 1)[::-1]
  V_sos_poly = prog.NewSosPolynomial(m)
  V = V_sos_poly[0].ToExpression()
  S = V_sos_poly[1]

  # Dynamics of the closed loop system (with infinite horizon LQR)
  f = quadrotor.closed_loop_dynamics_cubic_approximation(x)
  Vdot = V.Jacobian(x) @ f

  # Add SOS constraint and cost
  add_sos_constraint(prog, x, V, Vdot)
  add_cost(prog, S)

  # Solve the problem
  print("Start solving...")
  result = Solve(prog)

  # Retrieve the solution
  print(result.get_solution_result())
  print("S =", result.GetSolution(S))

  return result.GetSolution(S)

def visualize_region_of_attraction(S):
  '''
  Visualize the set V = x.T @ S @ x <= 1
  '''

  # Visualize the region of attraction
  N_grid = 100
  y_grid = np.linspace(-4, 4, N_grid)
  z_grid = np.linspace(-4, 4, N_grid)
  Y, Z = np.meshgrid(y_grid, z_grid)
  S_2d_grid = np.zeros((N_grid, N_grid))
  for i in range(N_grid):
    for j in range(N_grid):
      x = np.array([y_grid[i], z_grid[j], 0, 0, 0, 0])
      S_2d_grid[i, j] = x.T @ S @ x

  fig, ax = plt.subplots(figsize=(6,6))
  ax.contour(Y,Z, S_2d_grid, levels=[1])
  ax.set_xlabel("y (m)")
  ax.set_ylabel("z (m)")
  ax.set_title('Region of attraction (2D slice of a 6D space)')
  ax.set_aspect('equal')
  plt.show()

if __name__ == '__main__':
  R = np.eye(2);
  Q = np.diag([10, 10, 1, 1, 1, 1]);
  Qf = Q;
  quadrotor = Quadrotor(Q, R, Qf);

  S_sol = compute_region_of_attraction(quadrotor)
  np.save('S_sol', S_sol)
  visualize_region_of_attraction(S_sol)
