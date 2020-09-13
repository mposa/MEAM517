import numpy as np


class GridWorld():


  ''' 
  Gridworld dynamics very loosely borrowed from Sutton and Barto:
  Reinforcement Learning

  The dynamics have been simplified to be deterministic
  '''
  def __init__(self, slip_prob = 0.1):
    # actions legend 
    # 0: N 
    # 1: E 
    # 2: S 
    # 3: W 
    self.rows = 10
    self.cols = 10

    base_cost = 1
    large_cost = 5
    wall_cost = 2

    slow_states = [1 * self.rows + 2, 2 * self.rows + 2, 
                   3 * self.rows + 2, 4 * self.rows + 2,
                   5 * self.rows + 4, 5 * self.rows + 5, 
                   5 * self.rows + 6, 5 * self.rows + 7, 
                   5 * self.rows + 8]
    goal_state = 2 * self.rows + 4
                   
    P = np.zeros((self.rows * self.cols,4), dtype=(list))

    # TODO: Fill out the transition matrix P
    # P is a n x m matrix whose elements are a list of 2 tuples, the contents of 
    # each tuple is (probability, next_state, cost)
    #
    # The entry for the goal state has already been completed for you. You may 
    # find the convenience functions map_row_col_to_state() and 
    # map_state_to_row_col() helpful but are not required to use them.

    for s in range(self.rows * self.cols):
      if(s == goal_state):
        # Taking any action at the goal state will stay at the goal state
        P[s][0] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][1] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][2] = [(1.0, s, 0),
                   (0.0, 0, 0)]
        P[s][3] = [(1.0, s, 0),
                   (0.0, 0, 0)]

    self.P = P

  def map_row_col_to_state(self, row, col):
    return row * self.rows + col

  def map_state_to_row_col(self, state):
    return state // self.cols, np.mod(state, self.cols)

  def eval_action(self, state, action):
    row, col = self.map_state_to_row_col(state)
    if action < 0 or action > 3:
      raise ValueError('Not a valid action')
    if row < 0 or row >= self.rows:
      raise ValueError('Row out of bounds')
    if col < 0 or col >= self.cols:
      raise ValueError('Col out of bounds')
    return self.P[state, action]
