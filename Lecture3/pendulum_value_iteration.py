import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from discretize_system import discretize_second_order_system

def main():
    N = 101  # x-discretization
    M = 21  # u-discretization
    x_max = 2 * np.pi  # grid size for x and xdot
    xdot_max = 6;
    u_max = 3  # max input
    gamma = .995  # discount factor
    dt = 1e-3  # timestep size
    max_iter = 10000
    
    x = np.linspace(-x_max, x_max, N)
    xdot = np.linspace(-xdot_max, xdot_max, N)
    u = np.linspace(-u_max, u_max, M)

    X_grid = np.meshgrid(x, xdot)

    V = np.zeros(N*N)

    u_opt = np.zeros(N*N)

    # dynamics
    m = 1
    L = .5
    g = 9.8
    b = .1
    f = lambda x, xdot, u: (u-b*xdot - m*g*L*np.sin(x-np.pi))/(m * L**2)

    # running cost
    # cost = lambda x, xdot, u: x ** 2 + xdot ** 2 + u ** 2
    cost = lambda x, xdot, u: int(x**2 + xdot**2 > 0.05 )

    [T, C] = discretize_second_order_system(f, cost, x, xdot, u, dt)

    # Initialize error and iteration counter    
    error = 1
    iter = 0
    min_error = 1e-5

    fig = plt.figure()
    ax_1 = fig.add_subplot(2, 1, 1, projection='3d')
    ax_2 = fig.add_subplot(2, 1, 2, projection='3d')

    plt.ion()
    surf_1 = ax_1.plot_surface(X_grid[0], X_grid[1], V.reshape(N, N).T)
    surf_2 = ax_2.plot_surface(X_grid[0], X_grid[1], u_opt.reshape(N, N).T)

    # mng = plt.get_current_fig_manager()
    # mng.window.showMaximized()

    plt.show()


    while error > min_error and iter < max_iter:
        V_prev = np.copy(V)
        # Vectorized optimization. Reshapes V and C to represent states as a
        # single dimension
        V_update_by_cost = np.reshape(C + gamma*T.dot(V), [N*N, M])
            
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
            ax_2.set_zlabel('$u$', fontsize=16)

            plt.draw()
            plt.pause(0.001)

    print("Done! Total iterations: " + str(iter))
    input()
    plt.show()

    
if __name__ == "__main__":
    main()
