import numpy as np
import scipy as sp
import cvxpy as cp
import mosek

class LosslessSolver:

    def __init__(self, landing_point:np.ndarray, initial_state:np.ndarray, glide_slope:float, max_velocity:float, dry_mass:float, fuel_mass:float, alpha:float, lower_thrust_bound:float, upper_thrust_bound:float, tvc_range:float, delta_t:float, pointing_constraing:np.ndarray=np.array([0,0,1])):
        self.gravity_constant = 9.80665  # Gravitational acceleration in m/s^2
        self.gravity_vector = np.array([0, 0, -self.gravity_constant])  # Gravity vector in the world frame
        self.landing_point = landing_point # target point to land
        self.initial_state = initial_state  # State vector: [x, y, z, vx, vy, vz]
        self.glide_slope = glide_slope # makes the cone of allowed positions
        self.max_velocity = max_velocity # Maximum velocity in m/s (mostly to ensure we don't exceed the controllable flight envelope)
        self.tvc_range = tvc_range # Range of thrust vector control (TVC) angles in radians that are allowed to deviate from the desired direction (vertical)
        # attitude dynamics are not explicitly modeled since instead the attitude is enforced by the pointing constraint
        self.dry_mass = dry_mass  # Mass of the object
        self.fuel_mass = fuel_mass # Mass of fuel
        self.alpha = alpha  # Constant relating thrust to change in mass
        self.pointing_constraint = pointing_constraing  # Constraint on the pointing direction of the thrust vector in the body frame
        self.lower_thrust_bound = lower_thrust_bound  # Lower bound of thrust
        self.upper_thrust_bound = upper_thrust_bound # Upper bound of thrust
        self.delta_t = delta_t  # Time step for the solver

        # continuous-time state-space matrices
        self.A_c = np.array([0, 0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0])
        self.B_c = np.array([0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, -self.alpha])

    def update_parameters(self, landing_point:np.ndarray=None, initial_state:np.ndarray=None, glide_slope:float=None, max_velocity:float=None, dry_mass:float=None, fuel_mass:float=None, alpha:float=None, lower_thrust_bound:float=None, upper_thrust_bound:float=None, tvc_range:float=None, delta_t:float=None, pointing_constraing:np.ndarray=None):
        # Basically, only updates the parameters that are not None so we can choose what we want to update
        params = {
        'landing_point': landing_point,
        'initial_state': initial_state,
        'glide_slope': glide_slope,
        'max_velocity': max_velocity,
        'tvc_range': tvc_range,
        'dry_mass': dry_mass,
        'fuel_mass': fuel_mass,
        'alpha': alpha,
        'lower_thrust_bound': lower_thrust_bound,
        'upper_thrust_bound': upper_thrust_bound,
        'delta_t': delta_t,
        'pointing_constraing': pointing_constraing
        }
        for key, value in params.items():
            if value is not None:
                setattr(self, key, value)
                if key == 'alpha':
                    self.B_c = np.array([0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, -self.alpha])
        
    def compute_trajectory(self):
        # Discrete-time matrices
        A = sp.linalg.expm(self.A_c * self.delta_t)
        B = np.zeros_like(self.B_c)
        for i in range(self.B_c.shape[1]):
            for j in range(self.B_c.shape[0]):
                result, _ = sp.integrate.quad(lambda s: (sp.linalg.expm(self.A_c * (self.delta_t - s)) @ self.B_c)[j, i], 0, self.delta_t)
                B[j, i] = result

        t_min = self.dry_mass * np.linalg.norm(self.initial_state[3:6]) / self.upper_thrust_bound
        t_max = self.fuel_mass / (self.alpha * self.lower_thrust_bound)
        
        N_min = int(t_min / self.delta_t) + 1
        N_max = int(t_max / self.delta_t)

        # Here, we perform the line search to find the optimal N (number of time steps). Intuitively, this should be a convex
        # function, so we should be able to simply start at the minimum time and increase N until we find a feasible solution.
        # According to the 2007 paper, the optimal N is usually found to be close to N_min, and fuel cost is unimodal, for a 1-D simulation.
        for N in range(N_min, N_max + 1):
            pass

        '''
        the SOCP solver in MOSEK can only take in a list of single variables, so all lists will just have to be
        represented as sections ithin a larger sequence of variables. Since our problem is relaxed to be a single SOCP,
        we can just plug in Problem 4 from the 2007 paper and the thrust pointing constraints from the 2012 paper.
        The only problem is figuring out how to express this programatically such that the solver can understand it.
        '''
            
