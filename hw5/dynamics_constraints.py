import numpy as np

from pydrake.solvers.mathematicalprogram import MathematicalProgram
from pydrake.autodiffutils import AutoDiffXd


def EvaluateDynamics(planar_arm, context, x, u):
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
  h_i = np.zeros(4,)
  # TODO: Add a dynamics constraint using x_i, u_i, x_ip1, u_ip1, dt

  return h_i

def AddCollocationConstraints(prog, planar_arm, context, N, x, u, timesteps):
  # TODO: Add all the dynamics constraints (aka collocation constraints)
  #       to prog
  n_x = planar_arm.num_positions() + planar_arm.num_velocities()
  n_u = planar_arm.num_actuators()
  
  # Hint: use prog.AddConstraint(CollocationConstraintHelper, lb, ub, vars)
  # where vars = hstack(x[i], u[i], ...)
