import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def unicycle_input(t, y_spline, z_spline):
  #UNICYCLE_INPUT returns input to the unicycle
  #   @param t - current time
  #   @param y_spline - spline object for desired y trajectory
  #   @param z_spline - spline object for desired z trajectory
  #   
  #   @output u - input u(t) to the unicycle system

  # TODO: modify u to return the correct input for time t.
  u = np.zeros(2);


  return u
