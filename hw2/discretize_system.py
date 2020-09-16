import numpy as np
import scipy.sparse as sparse
from scipy.spatial import Delaunay

def discretize_second_order_system(f, cost, x, xdot, u, dt):
    # @param f The system dynamics, xddot = f(x,xdot,u)
    # @param cost The running cost, cost(x,xdot, u)
    # @param x A vector for the range of discretized states x
    # @param xdot A vector for the range of discretized states xdot
    # @param u A vector for the range of discretized inputs u
    # @param dt The sampling period
    # @output T The sparse transition matrix T is the probability
    #         of transitioning from x_i, xdot_j under action u_k to x_l, xdot_m.
    #         Indexing is T(i, j, k, l, m), but reshaped to be
    #         T(i * (N_xdot + M) + j * M + k, l * N_xdot + m)
    # @output C The cost matrix C(i, j, k) is the cost of taking action u_k from
    #           x_i, xdot_j, but reshaped to be
    #         C(i * (N_xdot + M) + j * M + k)
    # Note that the output reshaping greatly improves ultimate computational
    # efficiency and is necessary for the sparse storage of T

    N_x = len(x)
    N_xdot = len(xdot)
    M = len(u)

    C = np.zeros([N_x, N_xdot, M])
    # T = np.empty([N_x, N_xdot, M], dtype=sparse.coo_matrix)
    # T = np.zeros([N_x, N_xdot, M, N_x, N_xdot])

    T_rows = np.zeros(N_x * N_xdot * M * 3, dtype=np.int32)
    T_cols = np.zeros(N_x * N_xdot * M * 3, dtype=np.int32)
    T_data = np.zeros(N_x * N_xdot * M * 3)

    for i in range(N_x):
        x_i = x[i]
        for j in range(N_xdot):
            xdot_j = xdot[j]
            for k in range(M):
                ind = (i * N_xdot * M + j * M + k)

                u_k = u[k]
                # Evaluate cost
                C[i, j, k] = dt * cost(x_i, xdot_j, u_k)

                # Evaluate dynamics using forward Euler
                x_n = x_i + dt * xdot_j
                xdot_n = xdot_j + dt * f(x_i, xdot_j, u_k)

                # Barycentric interpolation
                [l, m, w] = barycentric(x, xdot, x_n, xdot_n)

                T_rows[ind * 3:ind * 3 + 3] = np.full(3, ind)
                T_cols[ind * 3:ind * 3 + 3] = l * N_xdot + m
                T_data[ind * 3:ind * 3 + 3] = w
                # T_cols[ind:ind+3] = [ind:ind+3]
                # T_cols[ind:ind+3] = [ind:ind+3]
                # T[i, j, k, l, m] = w
        # print(i)


    T = sparse.csr_matrix((T_data, (T_rows, T_cols)),
        shape=(N_x*N_xdot*M, N_x*N_xdot))
    C = C.reshape(N_x * N_xdot * M)
    return [T, C]

def barycentric(x, y, x_i, y_i):
    # @param x A vector for the x-dimension range
    # @param y A vector for the y-dimension range
    # @param x_i The x-value
    # @param y_i The y-value
    # @output x_inds The 3 x-indices for the interpolating points
    # @output y_inds The 3 y-indices for the interpolating points
    # @output w The 3 interpolating weights

    # Check boundaries
    if x_i >= x[-1] and y_i >= y[-1]:
        # second/third indices are dummy values
        x_inds = [len(x) - 1, len(x) - 2, len(x) - 3]
        y_inds = [len(y) - 1, len(y) - 2, len(y) - 3]
        weights = [1, 0, 0]
    elif x_i >= x[-1] and y_i <= y[0]:
        # second/third indices are dummy values
        x_inds = [len(x) - 1, len(x) - 2, len(x) - 3]
        y_inds = [0, 1, 2]
        weights = [1, 0, 0]
    elif x_i >= x[-1]:
        # third index is dummy value
        x_inds = [len(x) - 1, len(x) - 1, len(x) - 2]
        start_y = np.argmax(y >= y_i) - 1
        # z * y[start] + (1-z) * y[start+1] = y_i
        w_start = (y[start_y + 1] - y_i) / (y[start_y + 1] - y[start_y])
                
        y_inds = [start_y, start_y + 1, 0]
        weights = [w_start, 1 - w_start, 0]
    elif x_i <= x[0] and y_i >= y[-1]:
        # second/third indices are dummy values
        x_inds = [0, 1, 2]
        y_inds = [len(y) - 1, len(y) - 2, len(y) - 3]
        weights = [1, 0, 0]
    elif x_i <= x[0] and y_i <= y[0]:
        # second/third indices are dummy values
        x_inds = [0, 1, 2]
        y_inds = [0, 1, 2]
        weights = [1, 0, 0]
    elif x_i <= x[0]:
        # third index is dummy value
        x_inds = [0, 0, 1]
        start_y = np.argmax(y >= y_i) - 1
        # z * y[start] + (1-z) * y[start+1] = y_i
        w_start = (y[start_y + 1] - y_i) / (y[start_y + 1] - y[start_y])
                
        y_inds = [start_y, start_y + 1, 0]
        weights = [w_start, 1 - w_start, 0]
    elif y_i >= y[-1]:
        # third index is dummy value
        y_inds = [len(y) - 1, len(y) - 1, len(y) - 2]
        start_x = np.argmax(x >= x_i) - 1
        # z * x[start] + (1-z) * x[start+1] = x_i
        w_start = (x[start_x + 1] - x_i) / (x[start_x + 1] - x[start_x])
                
        x_inds = [start_x, start_x + 1, 0]
        weights = [w_start, 1 - w_start, 0]
    elif y_i <= y[0]:
        # third index is dummy value
        y_inds = [0, 0, 1]
        start_x = np.argmax(x >= x_i)  -1
        # z * x[start] + (1-z) * x[start+1] = x_i
        w_start = (x[start_x + 1] - x_i) / (x[start_x + 1] - x[start_x])
                
        x_inds = [start_x, start_x + 1, 0]
        weights = [w_start, 1 - w_start, 0]
    else:
        # Inside the range, perform full triangulation
        start_x = np.argmax(x >= x_i) - 1
        start_y = np.argmax(y >= y_i) - 1

        # determine which triangle in the box with lower-left corner
        # [start_x, start_y] we are in
        lx = x[start_x + 1] - x[start_x]
        dx = x_i - x[start_x]
        ly = y[start_y + 1] - y[start_y]
        dy = y_i - y[start_y]
        
        if dx * ly + dy * lx > lx * ly:
            # upper triangle
            x_inds = [start_x, start_x + 1, start_x + 1]
            y_inds = [start_y + 1, start_y + 1, start_y]
        else:
            # lower triangle
            x_inds = [start_x, start_x, start_x + 1]
            y_inds = [start_y + 1, start_y, start_y]
        # import pdb
        # pdb.set_trace()
        A = np.zeros([3,3])
        A[0,:] = x[x_inds]
        A[1,:] = y[y_inds]
        A[2,:] = np.ones(3)
        rhs = np.array([x_i, y_i, 1])
        weights = np.linalg.solve(A, rhs)
        

    return [np.array(x_inds), np.array(y_inds), np.array(weights)]
