import matplotlib.pyplot as plt
import numpy as np
from pydrake.solvers import MathematicalProgram, Solve
import pydrake.symbolic as sym

degree = 4

# Initialize an empty optimization program.
prog = MathematicalProgram()

# Declare "x" as indeterminates
x_ = prog.NewIndeterminates(1, "x")
x = x_[0]

gamma = prog.NewContinuousVariables(1, "gamma")[0]

sos_expression = sym.Polynomial(x**4 - 3*x**2 + 2*x + 1 - gamma, indeterminates=x_)
prog.AddSosConstraint(sos_expression)

prog.AddCost(-gamma)

# Solve the problem.
result = Solve(prog)

# Retrieve the solution.
print(result.get_solution_result())
print("gamma =", result.GetSolution(gamma))

x_eval = np.linspace(-2,2,1000)
y_eval = x_eval**4 - 3*x_eval**2 + 2*x_eval + 1


print("numerical min = ", np.min(y_eval))

plt.plot(x_eval, y_eval)
plt.show()