#include <iostream>
#include <cmath>
#include <math.h>

using namespace std;

int main() {
    // Static neutral state to compare to [pitch, yaw, roll]
    float base[3] = {0, 0, 1};

    // Desired orientation vector
    float desired[3] = {5, 2, 1}; 
    float mag = sqrt(pow(desired[0], 2) + pow(desired[1], 2) + pow(desired[2], 2));
    float desiredUnit[3] = {desired[0] / mag, desired[1] / mag, desired[2] / mag};

    // Angle calculations
    float yaw1 = atan2(desiredUnit[0], desiredUnit[2]);
    float pitch1 = atan2(desiredUnit[1], desiredUnit[2]);

    float yawDeg = yaw1 * 180 / M_PI;
    float pitchDeg = pitch1 * 180 / M_PI;

    // Convert yaw and pitch to pulse width signals
    float pulseYaw = 1500 + 10 * yawDeg;
    float puulsePitch = 1500 + 10 * pitchDeg;

    // Print pulse width outputs
    cout << pulseYaw;
    cout << "\n";
    cout << puulsePitch;

    return 0;
}