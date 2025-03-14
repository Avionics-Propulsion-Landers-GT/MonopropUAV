#ifndef LOOP_H
#define LOOP_H

#include <vector>
#include <string>
#include "init.h"
#include "Madgwick.h"
// #include "Matrix.h"
// #include "Vector.h"

struct LoopOutput {
    std::vector<std::vector<double>> state;   // Updated state (e.g., quaternion q)
    std::vector<bool> status;
    std::vector<double> command;         // Generated command based on state
};

void print(double value);
void preciseLatLonToMeters(double lat, double deltaLat, double deltaLon, double &dY, double &dX);
std::vector<double> weightedAverage(const std::vector<double>& v1, const std::vector<double>& v2, double weight1, double weight2);
LoopOutput loop(const std::vector<std::vector<double>>& values, const std::vector<std::vector<double>> state, SystemComponents& system, const std::vector<bool>& status, double dt);


#endif // LOOP_H