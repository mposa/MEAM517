import numpy as np
import sympy as sym
import matplotlib.pyplot as plt

# Calculate value function
# first, time to intersect the u=+/-1 curves
N = 50
x = np.linspace(-2, 2, N)
xdot = np.linspace(-2, 2, N)
X, XDOT = np.meshgrid(x, xdot)
# syms x xdot T
x_sym, xdot_sym, T = sym.symbols("x xdot T")

x_T1 = x_sym + xdot_sym*T + .5*T**2
xdot_T1 = xdot_sym + T
sol = sym.solvers.solve(xdot_T1**2 + 2*x_T1, T)
T1 = sol[1]
T2 = sym.simplify(xdot_T1.subs(T, T1))

V_sym = T1+T2

V_fun = sym.lambdify([x_sym, xdot_sym], V_sym)


V = np.zeros((N, N))
for i in range(N):
    for j in range(N):
        if XDOT[i, j] < -np.sign(X[i,j]) * np.sqrt(2 * np.abs(X[i,j])):
            V[i,j] = V_fun(X[i,j], XDOT[i,j])
        else:
            V[i,j] = V_fun(-X[i,j], -XDOT[i,j])



fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.plot_surface(X, XDOT, V)
ax.set_xlabel('x')
ax.set_ylabel('xdot')
ax.set_zlabel('minimum time')
plt.show()
