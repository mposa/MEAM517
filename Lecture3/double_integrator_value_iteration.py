import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from discretize_system import discretize_second_order_system

def main():
    N = 31  # x-discretization
    M = 11  # u-discretization
    x_max = 2  # grid size for x and xdot
    u_max = 1  # max input
    gamma = .995  # discount factor
    dt = .01  # timestep size
    max_iter = 10000
    
    x = np.linspace(-x_max, x_max, N)
    xdot = np.linspace(-x_max, x_max, N)
    u = np.linspace(-u_max, u_max, M)

    X_grid = np.meshgrid(x, xdot)

    V = np.zeros(N*N)

    u_opt = np.zeros(N*N)

    # dynamics
    f = lambda x, xdot, u: u

    # running cost
    # cost = lambda x, xdot, u: x ** 2 + xdot ** 2 + 2 * u ** 2
    cost = lambda x, xdot, u: int(x != 0 or xdot != 0)

    [T, C] = discretize_second_order_system(f, cost, x, xdot, u, dt)

    # Initialize error and iteration counter    
    error = 1
    iter = 0
    min_error = 1e-6

    fig_1 = plt.figure()
    ax_1 = fig_1.gca(projection='3d')

    fig_2 = plt.figure()
    ax_2 = fig_2.gca(projection='3d')

    plt.ion()
    surf_1 = ax_1.plot_surface(X_grid[0], X_grid[1], V.reshape(N, N).T)
    surf_2 = ax_2.plot_surface(X_grid[0], X_grid[1], u_opt.reshape(N, N).T)
    plt.show()

    # Reshape T to be (N*N*M, N*N) where first index represents current
    # state/action, and secon next state
    T_shape = T.reshape(N * N * M, N * N)
    
    # Reshape C to be (N*N*M, 1)
    C_shape = C.reshape(N * N * M)
    


    while error > min_error and iter < max_iter:
        V_prev = np.copy(V)
        # Vectorized optimization. Reshapes V and C to represent states as a
        # single dimension
        V_update_by_cost = np.reshape(
            C_shape + gamma*np.matmul(T_shape, V), [N*N, M])
            
        # Min over cost
        min_ind = np.argmin(V_update_by_cost, axis=1)
        V = V_update_by_cost[np.arange(V_update_by_cost.shape[0]), min_ind]
        u_opt = u[min_ind]
        
        iter = iter + 1
        error = np.max(np.abs(V_prev - V)) / np.max(V)

        if iter % 20 == 0:
            surf_1.remove()
            surf_1 = ax_1.plot_surface(X_grid[0], X_grid[1], V.reshape(N, N).T,
                                       cmap=cm.coolwarm)
            ax_1.set_xlabel('$x$', fontsize=16)
            ax_1.set_ylabel('$\dot x$', fontsize=16)
            ax_1.set_zlabel('$V$', fontsize=16)
            surf_2.remove()
            surf_2 = ax_2.plot_surface(X_grid[0], X_grid[1],
                                       u_opt.reshape(N, N).T,
                                       cmap=cm.coolwarm)
            ax_2.set_xlabel('$x$', fontsize=16)
            ax_2.set_ylabel('$\dot x$', fontsize=16)
            ax_1.set_zlabel('$u$', fontsize=16)
            plt.draw()
            plt.pause(0.001)

    print("Done! Total iterations: " + str(iter))
    input()
    plt.show()

    
if __name__ == "__main__":
    main()
