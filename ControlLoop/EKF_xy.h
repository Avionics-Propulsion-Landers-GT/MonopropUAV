#ifndef EKF_XY_H
#define EKF_XY_H

#include "ExtendedKalmanFilterGeneral.h"

/*

    Header file for EKF_xy

*/

class EKF_Position : public ExtendedKalmanFilterGeneral {
public:
    EKF_Position(const Eigen::VectorXd& initial_state, double delta_time, double q_scalar, double r_scalar, double initial_p);
    
    Eigen::VectorXd parseData(const std::vector<double>& data) override;
    Eigen::VectorXd stateTransitionFunction() override;
    Eigen::MatrixXd stateTransitionJacobian() override;
    Eigen::VectorXd measurementPredictionFunction() override;
    Eigen::MatrixXd measurementPredictionJacobian() override;

    Eigen::VectorXd getState() const { return state; } // Returns current state

};

#endif // EKF_XY_H
