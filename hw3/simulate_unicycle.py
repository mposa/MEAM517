from math import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import importlib

# Reload the module to use the latest code
import unicycle_spline
import unicycle_input
import plot_unicycle_trajectory
importlib.reload(unicycle_spline)
importlib.reload(unicycle_input)
importlib.reload(plot_unicycle_trajectory)
from unicycle_spline import unicycle_spline
from unicycle_input import unicycle_input
from plot_unicycle_trajectory import plot_unicycle_trajectory

def simulate_unicycle():
	# SIMULATE_UNICYCLE simulates a trajectory for the unicycle system
	t0 = 0.0
	tf = 10.0

	# get unicycle desired path
	y_spline, z_spline = unicycle_spline(t0, tf);

	# verify that \dot y > 0 for the given spline.
	t_chk = np.linspace(t0, tf, 1000)
	ydot_min = np.min(y_spline(t_chk, 1));
	if ydot_min <= 0:
		raise ValueError("ERROR: \\dot y is not positive over the entire trajectory!")

	# Autonomous ODE 
	def f(t, x):
		u = unicycle_input(t, y_spline, z_spline);
		xdot = np.array([np.cos(x[2])*u[1], 
			             np.sin(x[2])*u[1], 
			             u[0]])
		return xdot

	# Initial position
	x0 = np.array([0, 0, np.arctan2(z_spline(t0,1), y_spline(t0,1))])

	# Integrate 
	sol = solve_ivp(f, (0, tf), x0, t_eval=np.linspace(t0, tf, 100))

	return plot_unicycle_trajectory(sol.t, sol.y.T, y_spline, z_spline)

if __name__ == '__main__':
	simulate_unicycle()
