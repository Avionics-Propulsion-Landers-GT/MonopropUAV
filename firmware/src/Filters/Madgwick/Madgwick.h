#ifndef MADGWICK_H
#define MADGWICK_H

#include <vector>

/*

    Header file for updatedMadgwick.cpp filter.

*/

class Madgwick {
private:
    double gain;
    double beta;
    float lastUpdateTime;
    std::vector<double> eulerAttitudeEstimation;
    std::vector<double> orientation;

public:
    Madgwick();
    Madgwick(double setGain, double setBeta, double startTime, std::vector<double>& initialOrientation);

    std::vector<double> update(std::vector<double>& updateArr);
    std::vector<double> madgwickUpdate(std::vector<double>& q, 
                                       std::vector<double>& gyro, 
                                       std::vector<double>& accel, 
                                       std::vector<double>& mag, 
                                       double dt);

    void normalizeArray(std::vector<double>& array);
    double magnitude(std::vector<double>& array);
    std::vector<double> computeGradient(std::vector<double>& q, 
                                        std::vector<double>& accel, 
                                        std::vector<double>& mag);
    std::vector<double> quaternionToEuler(std::vector<double>& quaternion);
    std::vector<double> eulerToQuaternion(std::vector<double>& euler);
    std::vector<double> quaternionMultiply(std::vector<double>& q1, std::vector<double>& q2);
};

#endif // MADGWICK_H
