#ifndef EKF_GYRO_H
#define EKF_GYRO_H

#include "ExtendedKalmanFilterGeneral.h"
#include "../../CustomLinear/Quaternion.h"

class EKF_Gyro : public ExtendedKalmanFilterGeneral {
public:
    EKF_Gyro(const Vector& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p);

    Vector parseData(const std::vector<double>& data) override;
    Vector stateTransitionFunction(const Vector& angular_rates); // Relies on parsed data to update state
    Matrix stateTransitionJacobian(const Quaternion& orientation, const Vector& angular_rates);
    Vector measurementPredictionFunction() override;
    Matrix measurementPredictionJacobian() override;

    Vector getState() const { return state; } // Returns current state
};

#endif 
