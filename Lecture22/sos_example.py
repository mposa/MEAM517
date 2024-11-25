import numpy as np
from pydrake.solvers import MathematicalProgram, Solve
import pydrake.symbolic as sym

degree = 4

# Initialize an empty optimization program.
prog = MathematicalProgram()

# Declare "x" as indeterminates
x = prog.NewIndeterminates(2, "x")

V_sos, _ = prog.NewSosPolynomial(sym.Variables(x), degree)

V = V_sos

# V = V + sym.Polynomial(x.dot(x) * 1e-5)

# dynamics
x0_dot = sym.Polynomial(-x[1] + 1.5*x[0]**2 - .5*x[0]**3)
x1_dot = sym.Polynomial(3*x[0] - x[1])
x_dot = np.array([x0_dot, x1_dot])

V_dot = V.Jacobian(x).dot(x_dot)

# Vdot < 0
# prog.AddSosConstraint(-V_dot)
prog.AddSosConstraint(-V_dot - sym.Polynomial(x.dot(x) * 1e-5))

# V(0) = 0
env = {x[0]: 0,
       x[1]: 0}
prog.AddConstraint(V.EvaluatePartial(env).ToExpression() == 0)


# env = {x[0]: 1,
#        x[1]: 1}
# prog.AddConstraint(V.EvaluatePartial(env).ToExpression() == 1)


# Solve the problem.
result = Solve(prog)

# Retrieve the solution.
print(result.get_solution_result())
print("V =", result.GetSolution(V))

input()

print("V =", result.GetSolution(V).RemoveTermsWithSmallCoefficients(1e-6))