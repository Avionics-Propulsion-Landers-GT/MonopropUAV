#ifndef EXTENDED_KALMAN_FILTER_GENERAL_H
#define EXTENDED_KALMAN_FILTER_GENERAL_H

#include <Eigen/Dense>
#include <vector>

/*

    Header file for ExtendedKalmanFilterGeneral

*/

class ExtendedKalmanFilterGeneral {
protected:
    double previous_time;
    double current_time;
    double delta_time;
    Eigen::MatrixXd process_noise_covariance;
    Eigen::MatrixXd measurement_noise_covariance;
    Eigen::MatrixXd error_covariance;
    Eigen::VectorXd state;
    Eigen::VectorXd previous_state;

public:
    // Constructor
    ExtendedKalmanFilterGeneral(
        const Eigen::VectorXd& initial_state, 
        double delta_time, 
        double q_scalar, 
        double r_scalar, 
        double initial_p
    );

    // Virtual destructor
    virtual ~ExtendedKalmanFilterGeneral() {}

    Eigen::VectorXd getState() const; // Added get state method

    // Core EKF functions
    void predict();
    void update(const Eigen::VectorXd& measurement);

    // Abstract methods to be implemented by subclasses
    virtual Eigen::VectorXd parseData(const std::vector<double>& data) = 0;
    virtual Eigen::VectorXd stateTransitionFunction() = 0;
    virtual Eigen::MatrixXd stateTransitionJacobian() = 0;
    virtual Eigen::VectorXd measurementPredictionFunction() = 0;
    virtual Eigen::MatrixXd measurementPredictionJacobian() = 0;
};

#endif // EXTENDED_KALMAN_FILTER_GENERAL_H
