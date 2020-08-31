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
    # @output T The sparse transition matrix T(i, j, k, l, m) is the probability
    #         of transitioning from x_i, xdot_j under action u_k to x_l, xdot_m
    # @output C The cost matrix C(i, j, u) is the cost of taking action u_k from
    #           x_i, xdot_j
    #
    # Note: To use this in value iteration efficiently, it is probably a good
    # idea to reshape the outputs so that 
    #   T is (M*N^2 x N^2) and
    #   C is M*N^2 x 1

    N_x = len(x)
    N_xdot = len(xdot)
    M = len(u)

    C = np.zeros([N_x, N_xdot, M])
    # T = np.empty([N_x, N_xdot, M], dtype=sparse.coo_matrix)
    T = np.zeros([N_x, N_xdot, M, N_x, N_xdot])

    X_grid = np.array(np.meshgrid(x, xdot))
    grid_points = X_grid.reshape(-1, N_x * N_xdot).T
    tri = Delaunay(grid_points)
    N_tri = tri.transform.shape[0]
    tri_centroids = np.zeros([N_tri, 2])

    for i in range(N_tri):
        simplex = tri.simplices[i]
        tri_centroids[i] = np.sum(tri.points[simplex],axis=0) / 3

    for i in range(N_x):
        x_i = x[i]
        for j in range(N_xdot):
            xdot_j = xdot[j]
            for k in range(M):
                u_k = u[k]
                # Evaluate cost
                C[i, j, k] = dt * cost(x_i, xdot_j, u_k)

                # Evaluate dynamics using forward Euler
                x_n = x_i + dt * xdot_j
                xdot_n = xdot_j + dt * f(x_i, xdot_j, u_k)

                # Barycentric interpolation
                [l, m, w] = barycentric(tri, N_x, N_xdot, x_n, xdot_n)
                # T[i, j, k] = sparse.coo_matrix((w, (l, m)), shape=(N_x, N_xdot))
                T[i, j, k, l, m] = w
 
    return [T, C]
    
def barycentric(tri, N_x, N_y, x_i, y_i):
    # @param x_range A vector for the x-dimension range
    # @param y_range A vector for the y-dimension range
    # @param x The x-value
    # @param y The y-value
    # @output x_inds The 3 x-indices for the interpolating points
    # @output y_inds The 3 y-indices for the interpolating points
    # @output w The 3 interpolating weights
    # This function has not been optimized for performance!

    # Find the appropriate triangle and extract weights. Based on
    # https://scipy.github.io/devdocs/generated/scipy.spatial.Delaunay.html
    # but vectorized to compute over all triangles

    # By using `argmax` below, will choose the closest triangle if the point lies
    # outside all triangles
    p = np.array([(x_i, y_i)]).T

    b = np.einsum('ij,ikj->ik', (p.T - tri.transform[:, 2]), tri.transform[:,:2])
    weights = np.c_[b, 1 - np.sum(b, axis=1)]    
    index = np.argmax(np.min(weights, axis=1))

    w = weights[index]

    # If any element of w is less than 0, set to 0 and normalize
    if min(w) < 0:
        w[w < 0] = 0
        w = w / sum(w)
        
    # Extract x and y indices
    point_inds = tri.simplices[index]
    y_inds = (point_inds / N_x).astype(int)
    x_inds = point_inds % N_x

    return [x_inds, y_inds, w]