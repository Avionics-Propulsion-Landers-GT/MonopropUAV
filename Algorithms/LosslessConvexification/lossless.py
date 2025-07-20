import numpy as np
import scipy as sp
import cvxpy as cp

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
        self.A_c = np.array([[0, 0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0]])
        self.B_c = np.array([[0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [0, 0, 0, 0],
                       [1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, -self.alpha]])

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

        print(f"Time bounds: t_min = {t_min:.2f}s, t_max = {t_max:.2f}s")
        print(f"Number of time steps: N_min = {N_min}, N_max = {N_max}")

        # Line search for optimal N
        for N in range(N_min, N_max + 1):
            # Decision variables
            x = cp.Variable((7, N + 1))  # State vector [x, y, z, vx, vy, vz, m]
            u = cp.Variable((4, N))      # Control vector [Tx, Ty, Tz, T]

            constraints = []

            # Initial state constraint
            constraints.append(x[:, 0] == np.hstack((self.initial_state, self.dry_mass + self.fuel_mass)))

            # Dynamics constraints
            for k in range(N):
                constraints.append(x[:, k + 1] == A @ x[:, k] + B @ u[:, k])

            # Thrust constraints
            for k in range(N):
                constraints.append(cp.norm(u[:3, k], 2) <= u[3, k])  # Thrust vector magnitude
                constraints.append(self.lower_thrust_bound <= u[3, k])  # Lower bound
                constraints.append(u[3, k] <= self.upper_thrust_bound)  # Upper bound

            # Pointing constraints
            for k in range(N):
                constraints.append(cp.norm(u[:3, k] - u[3, k] * self.pointing_constraint, 2) <= u[3, k] * np.tan(self.tvc_range))
            
            # Glide slope constraint: keeps position within upward-opening cone
            # for k in range(N + 1):
            #     constraints.append(-x[2, k] <= self.glide_slope * cp.norm(x[:2, k], 2))
            for k in range(N + 1):
                constraints.append(cp.norm(x[:2, k], 2) <= (1 / self.glide_slope) * x[2, k])

            # Final state constraints
            constraints.append(x[:3, -1] == self.landing_point)  # Position
            constraints.append(x[3:6, -1] == 0)  # Velocity
            constraints.append(x[6, -1] >= self.dry_mass)  # Mass

            # Objective: Minimize fuel usage
            objective = cp.Minimize(cp.sum(u[3, :]) * self.delta_t)

            problem = cp.Problem(objective, constraints)
            try:
                result = problem.solve(solver=cp.ECOS)
                
                if problem.status == cp.OPTIMAL:
                    print(f"Optimal trajectory found with N = {N}")
                    return x.value, u.value
                elif problem.status == cp.INFEASIBLE:
                    print(f"Problem is infeasible with N = {N}")
                elif problem.status == cp.UNBOUNDED:
                    print(f"Problem is unbounded with N = {N}")
                else:
                    print(f"Optimization failed with status: {problem.status}")
                    
            except cp.error.SolverError as e:
                print(f"Solver Error: {e}")

        raise ValueError("No feasible solution found within the given range of N.")

        '''
        the SOCP solver in MOSEK can only take in a list of single variables, so all lists will just have to be
        represented as sections ithin a larger sequence of variables. Since our problem is relaxed to be a single SOCP,
        we can just plug in Problem 4 from the 2007 paper and the thrust pointing constraints from the 2012 paper.
        The only problem is figuring out how to express this programatically such that the solver can understand it.
        '''

if __name__ == "__main__":
    solver = LosslessSolver(
        landing_point=np.array([0, 0, 0]),
        initial_state=np.array([0, 0, 10, 0, 0, 0]),
        glide_slope=0.1,
        max_velocity=100,
        dry_mass=10,
        fuel_mass=60,
        alpha=0.01,
        lower_thrust_bound=1,
        upper_thrust_bound=2500,
        tvc_range=np.radians(10),
        delta_t=0.1,
        pointing_constraing=np.array([0, 0, 1])
    )
    trajectory, controls = solver.compute_trajectory()
    print("Optimal Trajectory:\n", trajectory)
    print("Optimal Controls:\n", controls)