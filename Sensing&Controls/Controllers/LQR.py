# Implementation of an LQR in Python
# Anyi Lin
import numpy as np
import time

'''
This creates an instance of an LQR controller. You can set all the matrices, and the input matrix should be made of
functions that take in 2 parameters: first - the state vector and second - the delta time.
'''
class LQR():
    '''
    Creates a new default LQR controller with all zeroed 1 size matrices.
    '''
    def __init__(self):
        self.a = np.zeros([1,1])
        self.b = np.zeros([1,1])
        self.q = np.zeros([1, 1])
        self.r = np.zeros([1, 1])
        self.x = np.zeros([1, 1])
        self.u = np.zeros([1, 1])
        self.last_update_time = time.perf_counter()
        self.delta_time = 0

    '''
    Creates a new LQR controller with an initial state vector and specified parameter matrices.
    '''
    def __init__(self, initial_state_x, set_q, set_r, set_a, set_b):
        self.a = set_a
        self.b = set_b
        self.q = set_q
        self.r = set_r
        self.x = initial_state_x
        self.u = np.zeros([self.r.shape[0],1])
        self.last_update_time = time.perf_counter()
        self.delta_time = 0
    
    ''''
    Sets the state matrix (a)
    '''
    def set_a(self, set_a):
        self.a = set_a
    
    ''''
    Sets the input matrix (b)
    '''
    def set_b(self, set_b):
        self.b = set_b
    
    ''''
    Sets the state cost matrix (q)
    '''
    def set_q(self, set_q):
        self.q = set_q
    
    ''''
    Sets the control cost matrix (r)
    '''
    def set_r(self, set_r):
        self.r = set_r

    ''''
    Sets the state vector
    '''
    def set_x(self, set_x):
        self.x = set_x

    '''
    This resets the state vector to zeros as well as the update timer
    '''
    def reset(self):
        self.reset(self, np.zeros([self.x.shape[0], 1]))

    '''
    This resets the state vector to some defined vector as well as the update timer
    '''
    def reset(self, set_x):
        self.x = set_x
        self.last_update_time = time.perf_counter()
    
    '''
    This runs one update of the discrete-time LQR controller for nonlinear systems.
    
    It computes the optimal control inputs given a nonlinear system, cost matrices, current state, and a target state.

    Parameters:
    :param current_state_x: The current state vector
    :param target_state_xf: The target state vector that we want to be at
    :param a: The state matrix. It describes how the system changes over time from its current state, without any inputs
    :param b: The input matrix. It describes how the system changes from inputs and is a function matrix
        - The A and B matrices are different for each system/inputs, and will need to be found, either experimentally
        or mathematically
    :param q: The state cost matrix. It determines how much penalty is given for each state variable in the cost function
    :param r: The input cost matrix. It determines how much penalty is given for each input variable (control effort) in the cost function
        - The Q and R matrices will also need to be tuned
    :param delta_time: The change in time between each time update is called, unless the delta time is reset
    :param N: The number of iterations the discrete-time Riccati equation uses. More iterations means better accuracy but more computational cost
    :param p: The solution to the discrete-time Riccati equation
    :param u: Contains all the control solutions from the discrete-time Riccati equation
    :param u_star: The last and best solution in u
    '''
    def update(self, current_state_x, target_state_xf):
        # Calculates the delta time between updates
        current_time = time.perf_counter()
        self.delta_time = current_time - self.last_update_time
        self.last_update_time = current_time

        # The b matrix evaluated so that it can be used in operations with other matrices
        B = np.zeros(self.b.shape)

        for i in range(self.b.shape[0]):
            for j in range(self.b.shape[1]):
                B[i][j] = self.b[i][j](self.x, self.delta_time)

        # Calculates how far off we are from the target state
        x_error = current_state_x - target_state_xf

        # Solves the discrete-time Riccati equation using N number of iterations
        N = 50
        p = [None] * (N + 1)
        qf = self.q
        p[N] = qf
        for i in range(N, 0, -1):
            p[i-1] = self.q + self.a.T @ p[i] @ self.a - (self.a.T @ p[i] @ B) @ np.linalg.pinv(
                self.r + B.T @ p[i] @ B) @ (B.T @ p[i] @ self.a)

        # Finds the optimal gain matrix and the optimal control vector iteravely
        k = [None] * N
        self.u = [None] * N
        for i in range(N):
            k[i] = -np.linalg.pinv(self.r + B.T @ p[i+1] @ B) @ B.T @ p[i+1] @ self.a
            self.u[i] = k[i] @ x_error
        u_star = self.u[N - 1]
        return u_star