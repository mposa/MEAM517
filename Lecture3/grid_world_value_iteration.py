import numpy as np
import matplotlib.pyplot as plt

def main():
    # Grid dimension
    N = 15
    M = 5

    use_obstacle = True
    # [xmin, xmax, ymin, ymax]
    obs_range = np.array([0, 5, 3, 5])

    # Build transition and cost matrix as MDP
    # T(x,y,a,xn,yn)
    #   actions are ordered: null, up, right, left, down
    #   actions that leave board are re-interpreted as null
    T = np.zeros([N, N, M, N, N])
    for i in range(N):
        for j in range(N):
            if (use_obstacle and i >= obs_range[0] and i <= obs_range[1] and
                    j >= obs_range[2] and j <= obs_range[3]):
                T[i, j,:, i, j] = np.ones(M)
                continue

            # null action
            T[i, j, 0, i, j] = 1

            # up action
            if i == 0:
                T[i, j, 1, i, j] = 1
            else:
                T[i, j, 1, i - 1, j] = 1
            
            # right action
            if j == N - 1:
                T[i, j, 2, i, j] = 1
            else:
                T[i, j, 2, i, j + 1] = 1

            # down action
            if i == N - 1:
                T[i, j, 3, i, j] = 1
            else:
                T[i, j, 3, i + 1, j] = 1
            
            # left action
            if j == 0:
                T[i, j, 4, i, j] = 1
            else:
                T[i, j, 4, i, j - 1] = 1


    # Build cost matrix
    C = np.ones([N, N, M])
    C[0, 0,:] = np.zeros([1, 1, M])
    

    # Reshape T to be (N*N*M, N*N) where first index represents current
    # state/action, and secon next state
    T_shape = T.reshape(N * N * M, N * N)
    
    # Reshape C to be (N*N*M, 1)
    C_shape = C.reshape(N * N * M)


    # Different possible initializations
    # V = 50 * np.random.rand(N * N)
    V = np.zeros([N*N ])
    # V = 50 * np.ones([N*N])

    V[0] = 0

    if use_obstacle:
        V.reshape(N,N)[obs_range[0]:obs_range[1]+1, obs_range[2]:obs_range[3]+1] = 100

    # Initialize error and iteration counter    
    error = 1
    iter = 0
    min_error = 1e-6
    max_iter = 100
    gamma = 1
    
    

    plt.ion()
    fig = plt.figure()
    image = plt.imshow(V.reshape(N, N).T, vmin=0, vmax=4*N)
    fig.colorbar(image)
    plt.draw()
    plt.pause(0.001)
    plt.show()

    while error > min_error and iter < max_iter:
        V_prev = np.copy(V)
        # Vectorized optimization. Reshapes V and C to represent states as a
        # single dimension
        V_update_by_cost = np.reshape(
            C_shape + gamma*np.matmul(T_shape, V), [N*N, M])
            
        # Min over cost
        min_ind = np.argmin(V_update_by_cost, axis=1)
        V = V_update_by_cost[np.arange(V_update_by_cost.shape[0]), min_ind]
        
        iter = iter + 1
        error = np.max(np.abs(V_prev - V)) / np.max(V)

        plt.imshow(V.reshape(N, N).T, vmin=0, vmax=4*N)
        plt.draw()
        plt.pause(0.001)

        iter = iter + 1

    input()


if __name__ == "__main__":
    main()