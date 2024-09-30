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

    def reset(self):
        self.x = np.zeros([self.x.shape[0], 1])
        self.last_update_time = time.perf_counter()

    def reset(self, set_x):
        self.x = set_x
        self.last_update_time = time.perf_counter()
    
    def update(self, current_state_x, target_state_xf):
        current_time = time.perf_counter()
        self.delta_time = current_time - self.last_update_time
        self.last_update_time = current_time

        B = np.zeros(self.b.shape)

        for i in range(self.b.shape[0]):
            for j in range(self.b.shape[1]):
                B[i][j] = self.b[i][j](self.x, self.delta_time)

        x_error = current_state_x - target_state_xf
        N = 50
        p = [None] * (N + 1)
        qf = self.q
        p[N] = qf
        for i in range(N, 0, -1):
            p[i-1] = self.q + self.a.T @ p[i] @ self.a - (self.a.T @ p[i] @ B) @ np.linalg.pinv(
                self.r + B.T @ p[i] @ B) @ (B.T @ p[i] @ self.a)
        k = [None] * N
        self.u = [None] * N
        for i in range(N):
            k[i] = -np.linalg.pinv(self.r + B.T @ p[i+1] @ B) @ B.T @ p[i+1] @ self.a
            self.u[i] = k[i] @ x_error
        u_star = self.u[N - 1]
        return u_star