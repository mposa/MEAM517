import matplotlib.pyplot as plt
import numpy as np
import importlib

from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from pydrake.all import (
    DiagramBuilder, Simulator
)

from pydrake.multibody.tree import (
    JointActuatorIndex
)

from pydrake.geometry import SceneGraph
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.trajectories import PiecewisePolynomial
from pydrake.solvers.snopt import SnoptSolver


import kinematic_constraints
import dynamics_constraints
importlib.reload(kinematic_constraints)
importlib.reload(dynamics_constraints)
from kinematic_constraints import (
  AddFinalLandingPositionConstraint
)
from dynamics_constraints import (
  AddCollocationConstraints,
  EvaluateDynamics
)


def find_throwing_trajectory(N, initial_state, distance, tf):
  '''
  Parameters:
    N - number of knot points
    initial_state - starting configuration
    distance - target distance to throw the ball

  '''

  builder = DiagramBuilder()
  plant = builder.AddSystem(MultibodyPlant(0.0))
  file_name = "planar_arm.urdf"
  Parser(plant=plant).AddModelFromFile(file_name)
  plant.Finalize()
  planar_arm = plant.ToAutoDiffXd()

  plant_context = plant.CreateDefaultContext()
  context = planar_arm.CreateDefaultContext()

  # Dimensions specific to the planar_arm
  n_x = planar_arm.num_positions() + planar_arm.num_velocities()
  n_u = planar_arm.num_actuators()

  # Store the actuator limits here
  effort_limits = np.zeros(n_u)
  for act_idx in range(n_u):
    effort_limits[act_idx] = \
      planar_arm.get_joint_actuator(JointActuatorIndex(act_idx)).effort_limit()
  vel_limits = 15 * np.ones(n_x // 2)

  # Create the mathematical program
  prog = MathematicalProgram()
  x = np.zeros((N, n_x), dtype="object")
  u = np.zeros((N, n_u), dtype="object")
  for i in range(N):
    x[i] = prog.NewContinuousVariables(n_x, "x_" + str(i))
    u[i] = prog.NewContinuousVariables(n_u, "u_" + str(i))

  t_land = prog.NewContinuousVariables(1, "t_land")

  t0 = 0.0
  timesteps = np.linspace(t0, tf, N)
  x0 = x[0]
  xf = x[-1]


  # Add the kinematic constraints (initial state, final state)
  # TODO: Add constraints on the initial state
  
  # Add the kinematic constraint on the final state
  AddFinalLandingPositionConstraint(prog, xf, distance, t_land)

  # Add the collocation aka dynamics constraints
  AddCollocationConstraints(prog, planar_arm, context, N, x, u, timesteps)

  # TODO: Add the cost function here

  # TODO: Add bounding box constraints on the inputs and qdot 

  # TODO: give the solver an initial guess for x and u using prog.SetInitialGuess(var, value)

  # Set up solver
  solver = SnoptSolver()
  result = solver.Solve(prog)
  
  x_sol = result.GetSolution(x)
  u_sol = result.GetSolution(u)
  t_land_sol = result.GetSolution(t_land)

  print(result.get_solution_result())

  # Reconstruct the trajecotry as a cubic hermite spline
  xdot_sol = np.zeros(x_sol.shape)
  for i in range(N):
    xdot_sol[i] = EvaluateDynamics(plant, plant_context, x_sol[i], u_sol[i])
  
  x_traj = PiecewisePolynomial.CubicHermite(timesteps, x_sol.T, xdot_sol.T)
  u_traj = PiecewisePolynomial.ZeroOrderHold(timesteps, u_sol.T)

  return x_traj, u_traj, prog, prog.GetInitialGuess(x), prog.GetInitialGuess(u)
  
if __name__ == '__main__':
  N = 7
  initial_state = np.zeros(6)
  tf = 5.0
  distance = 25.0
  x_traj, u_traj, prog, x_guess, u_guess = find_throwing_trajectory(N, initial_state, distance, tf)
