#ifndef LOOP_H
#define LOOP_H

#include "init.h"
#include "../Filters/Madgwick/Madgwick.h"
#include "../LQR/lqr.h"
#include "../CustomLinear/Matrix.h"

/*

    Header file for loop.cpp

*/

struct LoopOutput {
    std::vector<std::vector<double>> state;   // Updated state (e.g., quaternion q)
    std::vector<bool> status;
    std::vector<double> command;         // Generated command based on state
    std::vector<double> error;         // Error between state and setpoint
    std::vector<double> desired_command;
    std::vector<double> filteredCommand;
    double newX;
};

struct LoopInput {
    std::vector<std::vector<double>>& values;
    std::vector<std::vector<double>>& state;
    std::vector<std::vector<double>>& prevState;
    SystemComponents& system;
    std::vector<bool>& status;
    double dt;
    std::vector<double>& desired_state;
    std::vector<double>& delta_desired_state; 
    const std::vector<double>& command;
    const std::vector<double>& prevCommand; 
    const std::vector<double>& prevPrevCommand; 
    unsigned int& iter;
    double prevX;
};

void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double &dY, double &dX);
std::vector<double> weightedAverage(const std::vector<double>& v1, const std::vector<double>& v2, double weight1, double weight2);

//set point will vary and is subject to change during flight. It will be passed as a 1x12 vector
LoopOutput loop(LoopInput stuff);


#endif // LOOP_H