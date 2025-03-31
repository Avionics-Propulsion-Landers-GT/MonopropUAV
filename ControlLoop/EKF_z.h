#ifndef EKF_Z_H
#define EKF_Z_H

#include "ExtendedKalmanFilterGeneral.h"

/*

    Header file for EKF_Altitude (formerly EKF_z)

*/

class EKF_Altitude : public ExtendedKalmanFilterGeneral {
public:
    EKF_Altitude(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p);

    // Vector parseData(const std::vector<double>& data) override;
    Vector stateTransitionFunction() override;
    Matrix stateTransitionJacobian() override;
    Vector measurementPredictionFunction() override;
    Matrix measurementPredictionJacobian() override;

    Vector getState() const { return state; } // Returns current state
};

#endif // EKF_Z_H
