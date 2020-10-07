import numpy as np
from pydrake.autodiffutils import AutoDiffXd

from pydrake.solvers.mathematicalprogram import MathematicalProgram

def cos(theta):
  return AutoDiffXd.cos(theta)
def sin(theta):
  return AutoDiffXd.sin(theta)


def landing_constraint(vars):
  # import AutoDiffXd.cos as cos
  # import AutoDiffXd.sin as sin
  '''
  Impose a constraint such that if the ball is released at final state xf, 
  it will land a distance d from the base of the robot 
  '''
  l = 1
  g = 9.81
  constraint_eval = np.zeros((3,), dtype=AutoDiffXd)
  q = vars[:3]
  qdot = vars[3:6]
  t_land = vars[-1]
  pos = np.array([[l*sin(q[0]) + l*sin(q[0] + q[1]) + l*sin(q[0] + q[1] + q[2])],
                  [-l*cos(q[0]) - l*cos(q[0] + q[1]) - l*cos(q[0] + q[1] + q[2])]])
  vel = np.array([[l*qdot[2]*cos(q[0] + q[1] + q[2]) + qdot[0]*(l*cos(q[0]) + l*cos(q[0] + q[1]) + l*cos(q[0] + q[1] + q[2])) + qdot[1]*(l*cos(q[0] + q[1]) + l*cos(q[0] + q[1] + q[2]))],
                  [l*qdot[2]*sin(q[0] + q[1] + q[2]) + qdot[0]*(l*sin(q[0]) + l*sin(q[0] + q[1]) + l*sin(q[0] + q[1] + q[2])) + qdot[1]*(l*sin(q[0] + q[1]) + l*sin(q[0] + q[1] + q[2]))]])
  
  # TODO: Express the landing constraint as a function of q, qdot, and t_land

  return constraint_eval

def AddFinalLandingPositionConstraint(prog, xf, d, t_land):

  # TODO: Add the landing distance equality constraint as a system of inequality constraints 
  # using prog.AddConstraint(landing_constraint, lb, ub, vars) 

  # TODO: Add a constraint that t_land is positive

  pass
