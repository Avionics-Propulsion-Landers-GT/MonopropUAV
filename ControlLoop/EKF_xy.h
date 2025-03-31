#ifndef EKF_XY_H
#define EKF_XY_H

#include "ExtendedKalmanFilterGeneral.h"

/*

    Header file for EKF_Position (formerly EKF_xy)

*/

class EKF_Position : public ExtendedKalmanFilterGeneral {
public:
    EKF_Position(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p);

    // Vector parseData(const std::vector<double>& data) override;
    Vector stateTransitionFunction() override;
    Matrix stateTransitionJacobian() override;
    Vector measurementPredictionFunction() override;
    Matrix measurementPredictionJacobian() override;

    Vector getState() const { return state; } // Returns current state
};

#endif // EKF_XY_H
