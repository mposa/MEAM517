import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def unicycle_spline(t0, tf):
	#UNICYCLE_SPLINE returns a spline object representing a path from
	# (y(t0),z(t0)) = (0,0) to (y(t0),z(t0)) = (10,0) that avoids a circular
	# obstacle of radius 3 centered at (5,0), such that d\dt y(t) > 0
	#   @param t0 - initial time
	#   @param tf - final time
	#
	#   @output y_spline - spline object for desired y trajectory
	#   @output z_spline - spline object for desired z trajectory
	y0 = 0;
	z0 = 0;

	yf = 10;
	zf = 0;

	# TODO: design the spline here
	t = np.array([t0, tf])
	y = np.array([y0, yf])
	z = np.array([z0, zf])

	y_spline = CubicSpline(t, y);
	z_spline = CubicSpline(t, z);


	return y_spline, z_spline
