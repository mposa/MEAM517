from sympy import *
from sympy.utilities.lambdify import lambdastr
import numpy as np

def main():
  g = 9.81
  m = 1
  a = 0.25
  I = m*a**2


  y, z, theta, ydot, zdot, yddot, y3dot, y4dot, zddot, z3dot, z4dot, u1, u2 = symbols('y z theta ydot zdot yddot y3dot y4dot zddot z3dot z4dot u1 u2')

  # create lambda function for matlab function gradient
  gradient = lambda f, v: Matrix([f]).jacobian(v)
  theta = atan(-yddot/(zddot + g))
  b = gradient(theta,[y,ydot,yddot,y3dot,z,zdot,zddot,z3dot]).transpose()
  thetadot = simplify(gradient(theta,[y,ydot,yddot,y3dot,z,zdot,zddot,z3dot])@np.array([ydot,yddot,y3dot,y4dot,zdot,zddot,z3dot,z4dot]))
  thetaddot = simplify(gradient(thetadot,[y,ydot,yddot,y3dot,z,zdot,zddot,z3dot])@np.array([ydot,yddot,y3dot,y4dot,zdot,zddot,z3dot,z4dot]))

  f1 = -sin(theta)/m*(u1 + u2) - yddot
  f2  = a/I*(u1-u2) - thetaddot[0]
  # result = solve([f1,f2], (u1,u2))
  result = solve([f1,f2], u1, u2)
  import pdb; pdb.set_trace()
  p1 = result[u1]
  p2 = result[u2]

  t = symbols('t')
  c_x = 0
  c_y = 0
  R = 3
  omega = 1
  y_d = R*cos(omega*t) + c_x
  #y_d = t
  dy_d = diff(y_d,t)
  d2y_d = diff(dy_d,t)
  d3y_d = diff(d2y_d,t)
  d4y_d = diff(d3y_d,t)

  z_d = R*sin(omega*t) + c_y
  #z_d = t
  dz_d = diff(z_d,t)
  d2z_d = diff(dz_d,t)
  d3z_d = diff(d2z_d,t)
  d4z_d = diff(d3z_d,t)

  flat_out = [y, ydot, yddot, y3dot, y4dot, z, zdot, zddot, z3dot, z4dot]
  flat_out_d = [y_d, dy_d, d2y_d, d3y_d, d4y_d, z_d, dz_d, d2z_d, d3z_d, d4z_d]

  # theta_d = theta.subs(flat_out, flat_out_d)
  # dtheta_d = thetadot.subs(flat_out, flat_out_d)
  vars_to_sub = list(tuple(zip(flat_out, flat_out_d)))
  theta_d = theta.subs(vars_to_sub)
  dtheta_d = thetadot.subs(vars_to_sub)[0]
  u_1d = p1.subs(vars_to_sub)
  u_2d = p2.subs(vars_to_sub)

  x_d = [y_d, z_d, theta_d, dy_d, dz_d, dtheta_d]
  u_d = [u_1d, u_2d]

  theta, thetadot = symbols('theta thetadot')
  
  f = [ydot,
       zdot,
       thetadot,
       -sin(theta) * (u1 + u2) / m,
       -g + cos(theta) * (u1 + u2) / m,
       a * (u1 - u2) / I]
  x = [y, z, theta, ydot, zdot, thetadot]
  u = [u1, u2]
  A = Matrix([f]).jacobian(x).subs([x, u], [x_d, u_d]);
  B = Matrix([f]).jacobian(u).subs([x, u], [x_d, u_d]);
  f = Matrix([f]).subs([x, u], [x_d, u_d])
  
  import pdb; pdb.set_trace()
  print(lambdastr(t, x_d))
  print(lambdastr(t, u_d))
  # print(sympy.python(Matrix([x_d]), standard='C99'))

if __name__ == '__main__':
  main()
