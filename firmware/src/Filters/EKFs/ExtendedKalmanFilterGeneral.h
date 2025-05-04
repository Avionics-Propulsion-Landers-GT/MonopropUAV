#ifndef EXTENDED_KALMAN_FILTER_GENERAL_H
#define EXTENDED_KALMAN_FILTER_GENERAL_H

#include "../../CustomLinear/Matrix.h"
#include "../../CustomLinear/Vector.h"
#include <vector>

/*

    Header file for ExtendedKalmanFilterGeneral

*/

class ExtendedKalmanFilterGeneral {
protected:
    double previous_time;
    double current_time;
    double delta_time;

    Matrix process_noise_covariance;
    Matrix measurement_noise_covariance;
    Matrix error_covariance;

    Vector state;
    Vector previous_state;

public:
    // Constructor
    ExtendedKalmanFilterGeneral(
        const Vector& initial_state,
        double delta_time,
        double q_scalar,
        double r_scalar,
        double initial_p
    );

    // Virtual destructor
    virtual ~ExtendedKalmanFilterGeneral() {}

    Vector getState() const;

    // Core EKF functions
    void predict();
    void update(const Vector& measurement, const double delta_time);

    // Abstract methods to be implemented by subclasses
    virtual Vector parseData(const std::vector<double>& data) = 0;
    virtual Vector stateTransitionFunction() = 0;
    virtual Matrix stateTransitionJacobian() = 0;
    virtual Vector measurementPredictionFunction() = 0;
    virtual Matrix measurementPredictionJacobian() = 0;
};

#endif // EXTENDED_KALMAN_FILTER_GENERAL_H
