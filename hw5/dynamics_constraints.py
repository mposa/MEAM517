import numpy as np

from pydrake.solvers.mathematicalprogram import MathematicalProgram
from pydrake.autodiffutils import AutoDiffXd


def EvaluateDynamics(planar_arm, context, x, u):
  # Computes the dynamics xdot = f(x,u)

  planar_arm.SetPositionsAndVelocities(context, x)

  M = planar_arm.CalcMassMatrixViaInverseDynamics(context)
  B = planar_arm.MakeActuationMatrix()
  g = planar_arm.CalcGravityGeneralizedForces(context)
  C = planar_arm.CalcBiasTerm(context)

  M_inv = np.zeros((3,3))
  if(x.dtype == AutoDiffXd):
    M_inv = AutoDiffXd.inv(M)
  else:
    M_inv = np.linalg.inv(M)
  v_dot = M_inv @ (B @ u + g - C)
  return np.hstack((x[-3:], v_dot))

def CollocationConstraintEvaluator(planar_arm, context, dt, x_i, u_i, x_ip1, u_ip1):
  h_i = np.zeros(6,)
  # TODO: Evaluate the collocation constraint h using x_i, u_i, x_ip1, u_ip1, dt
  # You should make use of the EvaluateDynamics() function to compute f(x,u)

  return h_i

def AddCollocationConstraints(prog, planar_arm, context, N, x, u, timesteps):
  n_u = planar_arm.num_actuators()
  n_x = planar_arm.num_positions() + planar_arm.num_velocities()
  
  
  for i in range(N - 1):
    def CollocationConstraintHelper(vars):
      x_i = vars[:n_x]
      u_i = vars[n_x:n_x + n_u]
      x_ip1 = vars[n_x + n_u: 2*n_x + n_u]
      u_ip1 = vars[-n_u:]
      return CollocationConstraintEvaluator(planar_arm, context, timesteps[i+1] - timesteps[i], x_i, u_i, x_ip1, u_ip1)

    # TODO: Within this loop add the dynamics constraints for segment i (aka collocation constraints)
    #       to prog
    # Hint: use prog.AddConstraint(CollocationConstraintHelper, lb, ub, vars)
    # where vars = hstack(x[i], u[i], ...)


