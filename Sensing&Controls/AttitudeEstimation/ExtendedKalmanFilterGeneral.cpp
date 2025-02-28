#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>

class ExtendedKalmanFilterGeneral {
    protected:
        double previous_time;
        double current_time;
        double delta_time;
        Eigen::MatrixXd process_noise_covariance;
        Eigen::MatrixXd measurement_noise_covariance;
        Eigen::MatrixXd error_covariance;

    public:
    /*
    We first need an instantiatated constructor EKF with the following parameters:
        EKF (vector state, vector measurements, double deltaT, double qScale, double rScale, double initialP) {}

            Python Constructor (to be directly translated into C++): 
                def __init__(self, initial_state, initial_measurements, delta_time, q_scalar, r_scalar, initial_p):
                self.state = initial_state
                self.previous_state = initial_state
                self.previous_time = 0 - 2 * delta_time
                self.current_time = 0 - delta_time
                self.delta_time = delta_time
                self.measurement_size = np.size(initial_measurements)

                self.process_noise_covariance = np.eye(np.size(self.state)) * q_scalar
                self.measurement_noise_covariance = np.eye(self.measurement_size) * r_scalar

                self.error_covariance = np.eye((np.size(self.state))) * initial_p
    
    WE NEED THE FOLLOWING FUNCTIONS TO HAVE A DEFAULT IMPLEMENTATION IN THE GENERAL EKF CLASS
        update()
        predict()

    BELOW ARE PYTHON EQUIVALENT IMPLEMENTATIONS THAT WE NEED TO DIRECTLY TRANSLATE TO C++
    def update(self, data):
        # Update state estimate
        self.previous_state = self.state
        self.state = self.state_transition_function()

        # Predict covariance estimate
        self.error_covariance = self.state_transition_jacobian() @ self.error_covariance @ self.state_transition_jacobian().T + self.process_noise_covariance
    
    def predict(self):
        measurement = self.parse_data(data)

        # calculate measurement residual
        measurement_prediction = self.measurement_prediction_function()
        measurement_residual = measurement - measurement_prediction

        # calculate measurement residual covariance
        measurement_residual_covariance = self.measurement_prediction_jacobian() @ self.error_covariance @ self.measurement_prediction_jacobian().T + self.measurement_noise_covariance

        # calculate near-optimal kalman gain
        kalman_gain = self.error_covariance @ self.measurement_prediction_jacobian().T @ np.linalg.inv(measurement_residual_covariance)

        # update state estimate
        self.previous_state = self.state
        self.state = self.state + kalman_gain @ measurement_residual

        # update covariance estimate
        self.error_covariance = (np.eye(np.size(self.state)) - kalman_gain @ self.measurement_prediction_jacobian()) @ self.error_covariance


    WE NEED THE FOLLOWING METHODS AS ABSTRACT METHODS (FORCE CHILD CLASSES TO PROVIDE AN IMPLEMENTATION FOR THESE CLASSES):
        parseData()
        stateTransitionFunction()
        stateTransitionJacobian()
        measurementPredictionFunction()
        measurementPredictionJacobian()
    
    BELOW ARE THE PYTHON GENERAL IMPLEMENTATIONS FOR EACH (USE AS GUIDELINE TO UNDERSTAND DO NOT IMPLEMENT FOR THIS GENERAL EKF CLASS)
        def parse_data(self, data):
            self.previous_time = self.current_time
            self.current_time = data[0]
            self.delta_time = self.current_time - self.previous_time
            return np.array([data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]])

        def state_transition_function(self):
            self.velocity = (self.state - self.previous_state) / self.delta_time
            self.acceleration = (self.velocity - self.previous_velocity) / self.delta_time
            self.jerk = (self.acceleration - self.previous_acceleration) / self.delta_time
            return (self.state + (self.velocity*self.delta_time) + (self.acceleration * (0.5 * self.delta_time ** 2)) + ((1/6) * self.jerk * (self.delta_time ** 3)))
        
        def state_transition_jacobian(self):
            snap = (self.jerk - self.jerk)/self.delta_time
            if (self.velocity != 0):
                deriv = self.acceleration/self.velocity
                deriv2 = self.jerk/self.velocity
                deriv3 = snap/self.velocity
            else:
                deriv = 0
                deriv2 = 0
                deriv3 = 0
            jacobian = np.array([1 + deriv + deriv2 + deriv3])
            return jacobian
        
        def measurement_prediction_function(self):
            return self.state

        def measurement_prediction_jacobian(self):
            jacobian = np.ones((self.measurement_size, np.size(self.state)))
            return jacobian




    Create Own Quaternion, Matrix, and Vector (which should extend from matrix and is a Special case of matrix w/ 1 col) Classes
        preset matrix constructors (all zeros/identity matrix/all ones)
        general matrix operations we've used (inverse, transpose, multiplication)

    Math Helper Class containing at least the following methods: 
        def quaternions_to_euler_angular_velocities(self, q1, q2, dt):
            return 2 * self.quaternion_multiply(q2, self.quaternion_inverse(q1))[1:4] / dt

        # euler in the form (x,y,z)
        def euler_to_quaternion(self, euler):
            return np.array([math.cos(euler[0] / 2) * math.cos(euler[1] / 2) * math.cos(euler[2] / 2) + math.sin(euler[0] / 2) * math.sin(euler[1] / 2) * math.sin(euler[2] / 2),
                            math.sin(euler[0] / 2) * math.cos(euler[1] / 2) * math.cos(euler[2] / 2) - math.cos(euler[0] / 2) * math.sin(euler[1] / 2) * math.sin(euler[2] / 2),
                            math.cos(euler[0] / 2) * math.sin(euler[1] / 2) * math.cos(euler[2] / 2) + math.sin(euler[0] / 2) * math.cos(euler[1] / 2) * math.sin(euler[2] / 2),
                            math.cos(euler[0] / 2) * math.cos(euler[1] / 2) * math.sin(euler[2] / 2) - math.sin(euler[0] / 2) * math.sin(euler[1] / 2) * math.cos(euler[2] / 2)])
        
        # quaternion in the form (w,q1,q2,q3) where w is the real rotation component and q1,q2,q3 are the vector components
        def quaternion_to_euler(self, quaternion):
            return np.array([math.atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), (1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))),
                            math.asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1])),
                            math.atan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]), (1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])))])
        
        def quaternion_normalize(self, quaternion):
            norm = math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])
            if (norm == 0):
                return np.array([0, 0, 0, 0])
            return quaternion / norm
        
        def euler_normalize(self, euler):
            norm = math.sqrt(euler[0] * euler[0] + euler[1] * euler[1] + euler[2] * euler[2])
            if (norm == 0):
                return np.array([0, 0, 0])
            return euler / norm
        
        def quaternion_inverse(self, quaternion):
            norm = math.sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3])
            if (norm > 0):
                return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]) / norm
            return np.array([1, 0, 0, 0])
        
        def quaternion_multiply(self, q1, q2):
            return np.array([q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
                            q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
                            q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
                            q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]])
        
        #euler in (x,y,z)
        def get_extrinsic_rotation_matrix(self, euler):
            return self.get_extrinsic_z_rotation(euler[2]) @ self.get_extrinsic_y_rotation(euler[1]) @ self.get_extrinsic_x_rotation(euler[0])
        
        def get_extrinsic_x_rotation(self, x):
            return np.array([[1, 0, 0],
                            [0, math.cos(x), -math.sin(x)],
                            [0, math.sin(x), math.cos(x)]])
        
        def get_extrinsic_y_rotation(self, y):
            return np.array([[math.cos(y), 0, math.sin(y)],
                            [0, 1, 0],
                            [-math.sin(y), 0, math.cos(y)]])
        
        def get_extrinsic_z_rotation(self, z):
            return np.array([[math.cos(z), -math.sin(z), 0],
                            [math.sin(z), math.cos(z), 0],
                            [0, 0, 1]])
    
    */

    //Everything Below this should be updated
        ExtendedKalmanFilter(vector v, double delta_time, double q_scalar, double r_scalar, double initial_p);
        virtual ~ExtendedKalmanFilter() {}
    
        void predict();
        void update(const Eigen::VectorXd& measurement);

        virtual Eigen::MatrixXd parseData() = 0; 
        virtual Eigen::VectorXd stateTransitionFunction() = 0;
        virtual Eigen::MatrixXd stateTransitionJacobian() = 0;
        virtual Eigen::VectorXd measurementPredictionFunction() = 0;
        virtual Eigen::MatrixXd measurementPredictionJacobian() = 0;
};
