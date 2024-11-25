import numpy as np
from pydrake.solvers import MathematicalProgram, Solve
import pydrake.symbolic as sym
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

degree = 8

r_xT = .1 # radius for target set
r_X = 5

# Initialize an empty optimization program.
prog = MathematicalProgram()

n = 2
x = prog.NewIndeterminates(n, "x")

# dynamics
a = 1
x_dot = np.array([sym.Polynomial(-a*(x[0] - x[0]**3 / 3 - x[1])), sym.Polynomial(-x[0]/a)])

V = prog.NewFreePolynomial(sym.Variables(x), degree)
W, _ = prog.NewSosPolynomial(sym.Variables(x), degree, gram_name='W')
sigma_1, _ = prog.NewSosPolynomial(sym.Variables(x), degree, gram_name='S1')
sigma_2, _ = prog.NewSosPolynomial(sym.Variables(x), degree-2, gram_name='S2')
sigma_3, _ = prog.NewSosPolynomial(sym.Variables(x), degree-2, gram_name='S3')

# R^2 - ||x||^2
ball_constraint = sym.Polynomial(r_X**2 - x.dot(x))

# Vdot <= 0
V_dot = V.Jacobian(x).dot(x_dot)
prog.AddSosConstraint(-V_dot - ball_constraint * sigma_1)

# V >= 0 on target set
target_ball_constraint = sym.Polynomial(r_xT**2 - x.dot(x))
prog.AddSosConstraint(V - target_ball_constraint * sigma_2)

# W >= V + 1
prog.AddSosConstraint(W - V - 1 - ball_constraint * sigma_3)

# Create cost
N = 50
X0, X1 = np.meshgrid(np.linspace(-3,3,N), np.linspace(-3,3,N))
cost = np.zeros((N, N), sym.Polynomial)

for i, j in np.ndindex(X0.shape):
  env = {x[0]: X0[i, j], x[1]: X1[i,j]}
  cost[i, j] = W.EvaluatePartial(env)

prog.AddCost(np.sum(cost).ToExpression())


result = Solve(prog)
print(result.get_solution_result())

# Plotting
N = 41
X0, X1 = np.meshgrid(np.linspace(-3,3,N), np.linspace(-3,3,N))
X0_DOT = -a*(X0 - X0**3/3 - X1)
X1_DOT = -X0/a

V_plt = np.zeros((N, N))
V_sol = result.GetSolution(V)

W_plt = np.zeros((N, N))
W_sol = result.GetSolution(W)

for i, j in np.ndindex(X0.shape):
  env = {x[0]: X0[i, j], x[1]: X1[i,j]}
  V_plt[i, j] = V_sol.Evaluate(env)
  W_plt[i, j] = W_sol.Evaluate(env)

plt.quiver(X0, X1, X0_DOT, X1_DOT)
plt.contour(X0, X1, V_plt, [0], colors='red')

print("V =", result.GetSolution(V).RemoveTermsWithSmallCoefficients(1e-6))
print("W =", result.GetSolution(W).RemoveTermsWithSmallCoefficients(1e-6))

def f_vdp(t, x): return np.array([a*(x[0] - x[0]**3 / 3 - x[1]), +x[0]/a])
sol_ivp = solve_ivp(f_vdp, [0, 20], [0, 3], t_eval = np.linspace(0, 20, 500))
plt.plot(sol_ivp.y[0, 50:], sol_ivp.y[1,50:])


plt.figure()
cs = plt.contour(X0, X1, V_plt, 20)
plt.clabel(cs)
plt.title('V')

plt.figure()
cs = plt.contour(X0, X1, W_plt, 20)
plt.clabel(cs)
plt.title('W')



plt.show()


  