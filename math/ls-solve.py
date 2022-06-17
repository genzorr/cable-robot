import numpy as np
import qpsolvers

from qpsolvers import solve_ls, print_matrix_vector, solve_qp
from time import perf_counter

R = np.array([[3.23728,0,0,0,0,0,0],[0,3.23728,0,0,0,0,0],[0,0,3.23728,0,0,0,0],[0,0,0,3.23728,0,0,0],[0,0,0,0,1e-6,0,0],[0,0,0,0,0,1e-6,0],[0,0,0,0,0,0,1e-6]])
s = np.array([-15.8789,-15.8789,-15.8789,-15.8789,0,0,0])
G = np.array([[3.23728,0,0,0,0,0,0],[0,3.23728,0,0,0,0,0],[0,0,3.23728,0,0,0,0],[0,0,0,3.23728,0,0,0],[0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1],[-3.23728,0,0,0,0,0,0],[0,-3.23728,0,0,0,0,0],[0,0,-3.23728,0,0,0,0],[0,0,0,-3.23728,0,0,0],[0,0,0,0,-1,0,0],[0,0,0,0,0,-1,0],[0,0,0,0,0,0,-1]])
h = np.array([84.1211,84.1211,84.1211,84.1211,0.1,0.1,0.1,15.8789,15.8789,15.8789,15.8789,0.1,0.1,0.1])
A = np.array([[-1.8,-1.8,1.8,1.8,-0,0,0],[-1.8,1.8,1.8,-1.8,0,-0,0],[2,2,2,2,0,0,0],[-0.4,0.4,0.4,-0.4,-0.741,-0,-0],[0.4,0.4,-0.4,-0.4,-0,-0.741,0],[0,0,0,0,-0,0,-1.391]])
b = np.array([0,0,-3.55271e-15,0,0,0])


if __name__ == "__main__":
    start_time = perf_counter()
    solver = "quadprog"  # see qpsolvers.available_solvers
    print(qpsolvers.available_solvers)
    # x = solve_ls(R, s, G, h, A, b, solver=solver, verbose=True)
    P = np.dot(R.T, R)
    q = np.dot(s, R).reshape((len(s),))
    x = solve_qp(P, q, G, h, A, b, solver=solver, verbose=True)
    end_time = perf_counter()

    # print("")
    # print("    min. || R * x - s ||^2")
    # print("    s.t. G * x <= h")
    # print("")
    # print_matrix_vector(R, "R", s, "s")
    # print("")
    # print_matrix_vector(G, "G", h, "h")
    # print("")
    # print_matrix_vector(A, "A", b, "b")
    # print("")
    print(f"Solution: x = {x}")
    print(f"Solve time: {1e6 * (end_time - start_time):.0f} [us]")
    print(f"Solver: {solver}")