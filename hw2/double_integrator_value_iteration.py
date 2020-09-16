import importlib

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import discretize_system
importlib.reload(discretize_system)
from discretize_system import discretize_second_order_system
from mpl_toolkits.mplot3d import Axes3D

from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation

def run_iteration():
    # Default resolution
    N = 31  # x-discretization
    M = 21  # u-discretization

    # Modify the resolution here
    # 
    # N = 11 
    # M = 5 

    # Analytical solutions of value function and optimal policy 
    V_star = np.zeros((N,N))
    u_star = np.zeros((N,N))
    # TODO: Fill in V_star and u_star with the analytical solution derived in part a


    # Parameters
    x_max = 2  # grid size for x and xdot
    u_max = 1  # max input
    gamma = .995  # discount factor
    dt = .01  # timestep size
    max_iter = 10000
    
    x = np.linspace(-x_max, x_max, N)
    xdot = np.linspace(-x_max, x_max, N)
    u = np.linspace(-u_max, u_max, M)

    X_grid = np.meshgrid(x, xdot)

    # Dynamics
    f = lambda x, xdot, u: u

    # Running cost
    Q = np.array([[5, 1],
                  [1, 1]])
    cost = lambda x, xdot, u: 0.5 * (np.array([x, xdot]) @ Q @ np.array([x, xdot]).T + u ** 2)

    [T, C] = discretize_second_order_system(f, cost, x, xdot, u, dt)

    # Initilaize value function and optimal policy for value iteration
    V = np.zeros(N*N)
    u_opt = np.zeros(N*N)

    # Initialize error and iteration counter    
    error = 1
    iter = 0
    min_error = 1e-5

    # Value iterations 
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
    print("Done! Total iterations: " + str(iter))

    ###
    ### Create plots
    ###
    fig1 = plt.figure(r'$V$')
    ax_1 = fig1.add_subplot(1, 1, 1, projection='3d')
    fig2 = plt.figure(r'$u$')
    ax_2 = fig2.add_subplot(1, 1, 1, projection='3d')
    fig3 = plt.figure(r'$V - V^*$')
    ax_3 = fig3.add_subplot(1, 1, 1, projection='3d')
    fig4 = plt.figure(r'$u - u^*$')
    ax_4 = fig4.add_subplot(1, 1, 1, projection='3d')

    surf_1 = ax_1.plot_surface(X_grid[0], X_grid[1], V.reshape(N, N).T,
                                cmap=cm.coolwarm)
    surf_2 = ax_2.plot_surface(X_grid[0], X_grid[1],
                                u_opt.reshape(N, N).T,
                                cmap=cm.coolwarm)
    surf_3 = ax_3.plot_surface(X_grid[0], X_grid[1],
                                V.reshape(N, N).T - V_star.T,
                                cmap=cm.coolwarm)
    surf_4 = ax_4.plot_surface(X_grid[0], X_grid[1],
                                u_opt.reshape(N, N).T - u_star,
                                cmap=cm.coolwarm)

    ax_1.set_xlabel('$x$', fontsize=16)
    ax_1.set_ylabel('$\dot x$', fontsize=16)
    ax_1.set_zlabel('$V$', fontsize=16)
    ax_2.set_xlabel('$x$', fontsize=16)
    ax_2.set_ylabel('$\dot x$', fontsize=16)
    ax_2.set_zlabel('$u$', fontsize=16)
    ax_3.set_xlabel('$x$', fontsize=16)
    ax_3.set_ylabel('$\dot x$', fontsize=16)
    ax_3.set_zlabel('$V - V^*$', fontsize=16)
    ax_4.set_xlabel('$x$', fontsize=16)
    ax_4.set_ylabel('$\dot x$', fontsize=16)
    ax_4.set_zlabel('$u - u^*$', fontsize=16)

    plt.show();

    # Save the plots for submission
    fig3.savefig("dV.png", dpi=240)
    fig4.savefig("du.png", dpi=240)

    ###
    ### Create animation
    ###
    from matplotlib import rc
    rc('animation', html='jshtml')

    fig5 = plt.figure(r'$u - u^*$')
    ax_5 = fig5.add_subplot(1, 1, 1, projection='3d')
    def frame5(i):
      ax_5.clear()
      plot = ax_5.plot_surface(X_grid[0], X_grid[1], V.reshape(N, N).T,
                                cmap=cm.coolwarm)
      ax_5.view_init(30, i * 10)
      ax_5.set_xlabel('$x$', fontsize=16)
      ax_5.set_ylabel('$\dot x$', fontsize=16)
      ax_5.set_zlabel('$V$', fontsize=16)
      return plot
    anim1 = animation.FuncAnimation(fig5, frame5, frames=18, blit=False, repeat=False)
    plt.close()

    fig6 = plt.figure(r'$u - u^*$')
    ax_6 = fig6.add_subplot(1, 1, 1, projection='3d')
    def frame6(i):
      ax_6.clear()
      plot = ax_6.plot_surface(X_grid[0], X_grid[1],
                                u_opt.reshape(N, N).T,
                                cmap=cm.coolwarm)
      ax_6.view_init(30, i * 10)
      ax_6.set_xlabel('$x$', fontsize=16)
      ax_6.set_ylabel('$\dot x$', fontsize=16)
      ax_6.set_zlabel('$u$', fontsize=16)
      return plot
    anim2 = animation.FuncAnimation(fig6, frame6, frames=18, blit=False, repeat=False)
    plt.close()

    fig7 = plt.figure(r'$u - u^*$')
    ax_7 = fig7.add_subplot(1, 1, 1, projection='3d')
    def frame7(i):
      ax_7.clear()
      plot = ax_7.plot_surface(X_grid[0], X_grid[1],
                                V.reshape(N, N).T - V_star.T,
                                cmap=cm.coolwarm)
      ax_7.view_init(30, i * 10)
      ax_7.set_xlabel('$x$', fontsize=16)
      ax_7.set_ylabel('$\dot x$', fontsize=16)
      ax_7.set_zlabel('$V - V^*$', fontsize=16)
      return plot
    anim3 = animation.FuncAnimation(fig7, frame7, frames=18, blit=False, repeat=False)
    plt.close()

    fig8 = plt.figure(r'$u - u^*$')
    ax_8 = fig8.add_subplot(1, 1, 1, projection='3d')
    def frame8(i):
      ax_8.clear()
      plot = ax_8.plot_surface(X_grid[0], X_grid[1],
                                  u_opt.reshape(N, N).T - u_star,
                                  cmap=cm.coolwarm)
      ax_8.view_init(30, i * 10)
      ax_8.set_xlabel('$x$', fontsize=16)
      ax_8.set_ylabel('$\dot x$', fontsize=16)
      ax_8.set_zlabel('$u - u^*$', fontsize=16)
      return plot
    anim4 = animation.FuncAnimation(fig8, frame8, frames=18, blit=False, repeat=False)
    plt.close()
    
    return anim1, anim2, anim3, anim4

if __name__ == "__main__":
    anim1, anim2, anim3, anim4 = run_iteration()
