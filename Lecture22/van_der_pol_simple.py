import numpy as np
from pydrake.solvers import MathematicalProgram, Solve, SolverOptions, ClarabelSolver
import pydrake.symbolic as sym
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# degree = 4
# sigma_degree = 4

degree = 8
sigma_degree = 12

# Initialize an empty optimization program.
prog = MathematicalProgram()

n = 2
x = prog.NewIndeterminates(n, "x")

# dynamics
a = 1
x_dot = np.array([sym.Polynomial(-a*(x[0] - x[0]**3 / 3 - x[1])), sym.Polynomial(-x[0]/a)])

V, _ = prog.NewSosPolynomial(sym.Variables(x), degree)

V_dot = V.Jacobian(x).dot(x_dot)

# SOS constraints
sigma_1 = sym.Polynomial((x.dot(x))**(sigma_degree/2))
sigma_2 = sym.Polynomial((1 + x.dot(x))**(sigma_degree/2 - 1))

V_dot_sos = sigma_1*(V - 1) - sigma_2*V_dot
prog.AddSosConstraint(V_dot_sos)

# Create cost
N = 5
X0, X1 = np.meshgrid(np.linspace(-3,3,N), np.linspace(-3,3,N))
cost = np.zeros((N, N), sym.Polynomial)

for i, j in np.ndindex(X0.shape):
  env = {x[0]: X0[i, j], x[1]: X1[i,j]}
  cost[i, j] = V.EvaluatePartial(env)

prog.AddCost(np.sum(cost).ToExpression())


# V(0) = 0
env = {x[0]: 0,
       x[1]: 0}
prog.AddConstraint(V.EvaluatePartial(env).ToExpression() == 0)

# import pdb; pdb.set_trace()
options = SolverOptions()
options.SetOption(ClarabelSolver.id(), 'verbose', True)
options.SetOption(ClarabelSolver.id(), 'tol_gap_abs', 1e-7)
options.SetOption(ClarabelSolver.id(), 'tol_gap_rel', 1e-7)
options.SetOption(ClarabelSolver.id(), 'tol_feas', 1e-7)
options.SetOption(ClarabelSolver.id(), 'tol_infeas_abs', 1e-7)
options.SetOption(ClarabelSolver.id(), 'tol_infeas_rel', 1e-7)
options.SetOption(ClarabelSolver.id(), 'tol_ktratio', 1e-5)

prog.SetSolverOptions(options)
result = Solve(prog)
print(result.get_solver_details())
print(result.get_solution_result())

# Plotting
N = 40
X0, X1 = np.meshgrid(np.linspace(-3,3,N), np.linspace(-3,3,N))
X0_DOT = -a*(X0 - X0**3/3 - X1)
X1_DOT = -X0/a

V_plt = np.zeros((N, N))
V_sol = result.GetSolution(V)
for i, j in np.ndindex(X0.shape):
  env = {x[0]: X0[i, j], x[1]: X1[i,j]}
  V_plt[i, j] = V_sol.Evaluate(env)

plt.quiver(X0, X1, X0_DOT, X1_DOT)
plt.contour(X0, X1, V_plt, [1], colors='red')



def f_vdp(t, x): return np.array([a*(x[0] - x[0]**3 / 3 - x[1]), +x[0]/a])
sol_ivp = solve_ivp(f_vdp, [0, 20], [0, 3], t_eval = np.linspace(0, 20, 500))
plt.plot(sol_ivp.y[0, 50:], sol_ivp.y[1,50:])

plt.show()
