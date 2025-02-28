#include <iostream>
#include <vector>
#include <tuple>
#include <map>
#define _USE_MATH_DEFINES
#include <cmath>

// Constants
const double max_tilt = 30.0;          // Maximum angle that rocket could tilt (degrees)
const double max_tvc = 15.0;           // Maximum angle that the TVC can achieve (degrees)
const double max_speed = 22.5;         // Maximum reachable speed for any arbitrary point
const double max_acceleration = 2.25;  // Maximum reachable acceleration for any arbitrary point
const double MAX_ANGLE = 13.5;         // Maximum angle between positions


// TODO: Might need to refactor Point3D to Vectors, for ease of use. Unsure how Point3D will affect future logic.

// Type definitions
using ArrayOfPoints = std::vector<double>;
using ArrayOfVelocities = std::vector<double>;

// Struct to represent a 3D point or vector
struct Point3D {
    double x;
    double y;
    double z;
};

// Bezier curve function
Point3D bezier_curve(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t) {
    double one_minus_t = 1.0 - t;
    double one_minus_t_sq = one_minus_t * one_minus_t;
    double one_minus_t_cu = one_minus_t_sq * one_minus_t;
    double t_sq = t * t;
    double t_cu = t_sq * t;

    Point3D result;
    result.x = one_minus_t_cu * p0.x + 3 * one_minus_t_sq * t * p1.x + 3 * one_minus_t * t_sq * p2.x + t_cu * p3.x;
    result.y = one_minus_t_cu * p0.y + 3 * one_minus_t_sq * t * p1.y + 3 * one_minus_t * t_sq * p2.y + t_cu * p3.y;
    result.z = one_minus_t_cu * p0.z + 3 * one_minus_t_sq * t * p1.z + 3 * one_minus_t * t_sq * p2.z + t_cu * p3.z;

    return result;
}

// First derivative of Bezier curve
Point3D first_derivative(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t) {
    double t_sq = t * t;

    double coef0 = (-3 * t_sq) + (6 * t) - 3;
    double coef1 = (9 * t_sq) - (12 * t) + 3;
    double coef2 = (-9 * t_sq) + (6 * t);
    double coef3 = 3 * t_sq;

    Point3D result;
    result.x = coef0 * p0.x + coef1 * p1.x + coef2 * p2.x + coef3 * p3.x;
    result.y = coef0 * p0.y + coef1 * p1.y + coef2 * p2.y + coef3 * p3.y;
    result.z = coef0 * p0.z + coef1 * p1.z + coef2 * p2.z + coef3 * p3.z;

    return result;
}

// Second derivative of Bezier curve
Point3D second_derivative(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t) {
    double coef0 = (-6 * t) + 6;
    double coef1 = (18 * t) - 12;
    double coef2 = (-18 * t) + 6;
    double coef3 = 6 * t;

    Point3D result;
    result.x = coef0 * p0.x + coef1 * p1.x + coef2 * p2.x + coef3 * p3.x;
    result.y = coef0 * p0.y + coef1 * p1.y + coef2 * p2.y + coef3 * p3.y;
    result.z = coef0 * p0.z + coef1 * p1.z + coef2 * p2.z + coef3 * p3.z;

    return result;
}

// Hermite curvature (unused in the main code)
std::tuple<double, double, double> hermite_curvature(double x1, double y1, double z1, double x2, double y2, double z2) {
    double n0 = x1 * y2 - y1 * x2;
    double n1 = y1 * z2 - z1 * y2;
    double n2 = z1 * x2 - x1 * z2;
    double d0 = std::pow((x1 * x1) + (y1 * y1), 1.5);
    double d1 = std::pow((y1 * y1) + (z1 * z1), 1.5);
    double d2 = std::pow((z1 * z1) + (x1 * x1), 1.5);
    return std::make_tuple(n0 / d0, n1 / d1, n2 / d2);
}

// Check the angle between two points to see if the endpoint is obtainable
bool angle_check(const Point3D& p0, const Point3D& p1) {
    double adj = std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));
    double opp = p1.z - p0.z;
    double angle = std::atan2(opp, adj) * 180.0 / M_PI;
    // We use MAX_ANGLE to give the rocket some leeway
    return angle >= (90.0 - MAX_ANGLE);
}

// Get a new endpoint if the original is not obtainable
Point3D get_new_endpoint(const Point3D& p0) {
    // The endpoint wants to be above the origin (0, 0, z)
    double adj = std::sqrt(p0.x * p0.x + p0.y * p0.y);
    double z = adj * std::tan((90.0 - MAX_ANGLE) * M_PI / 180.0);
    Point3D p1 = {0.0, 0.0, z};
    return p1;
}

// Find the magnitude of a vector
double get_mag(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
}

// Get the maximum curvature and its index
std::pair<double, double> get_max_curve(const std::vector<double>& c) {
    double max_val = c[0];
    int idx = 0;
    for (int i = 0; i < 100; ++i) {
        if (c[i] > max_val) {
            max_val = c[i];
            idx = i;
        }
    }
    double t_value = static_cast<double>(idx) / 100.0;
    return std::make_pair(max_val, t_value);
}

// Create Bezier curve points
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> createPoints(Point3D p0, Point3D v0, Point3D& p1, int num_points) {
    Point3D p1_og = p1;
    bool new_point = false;

    if (!angle_check(p0, p1)) {
        p1 = get_new_endpoint(p0);
        new_point = true;
    }

    // Calculate the two Bezier control points using the initial conditions and endpoint
    Point3D b0 = {p0.x + v0.x / 3.0, p0.y + v0.y / 3.0, p0.z + v0.z / 3.0};
    Point3D b1 = {p1.x, p1.y, p0.z + (p1.z - p0.z) / 3.0};

    std::vector<double> x, y, z;

    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        Point3D point = bezier_curve(p0, b0, b1, p1, t);
        x.push_back(point.x);
        y.push_back(point.y);
        z.push_back(point.z);
    }

    return std::make_tuple(x, y, z);
}

// Create target tangent vectors (unused in the main code)
std::map<std::tuple<double, double, double>, Point3D> createTargetTangentVectors(const ArrayOfPoints& x_arr, const ArrayOfPoints& y_arr, const ArrayOfPoints& z_arr) {
    std::map<std::tuple<double, double, double>, Point3D> velocities; // Dictionary where points are keys and values are the unit tangent vector at that point
    for (size_t i = 0; i < x_arr.size() - 1; ++i) {
        double vx = x_arr[i + 1] - x_arr[i];
        double vy = y_arr[i + 1] - y_arr[i];
        double vz = z_arr[i + 1] - z_arr[i];
        double mag = get_mag(vx, vy, vz);
        Point3D unit_v = {vx / mag, vy / mag, vz / mag};
        velocities[std::make_tuple(x_arr[i], y_arr[i], z_arr[i])] = unit_v;
    }
    return velocities;
}

int main() {
    // Setup initial values and arrays to store points
    Point3D p0 = {7.0, 4.0, 0.0};
    Point3D v0 = {3.0, 5.0, 12.0};
    Point3D p1 = {0.0, 0.0, 25.0};
    Point3D p1_og = p1;
    bool new_point = true;

    // angle check is enabled for cpp code. How to fix?

    // Create points along the Bezier curve
    int num_points = 100;
    std::vector<double> x, y, z;
    std::tie(x, y, z) = createPoints(p0, v0, p1, num_points);

    // Calculate the two Bezier control points using the initial conditions and endpoint
    Point3D b0 = {p0.x + v0.x / 3.0, p0.y + v0.y / 3.0, p0.z + v0.z / 3.0};
    Point3D b1 = {p1.x, p1.y, p0.z + (p1.z - p0.z) / 3.0};

    // Initialize arrays for velocities, accelerations, etc.
    std::vector<double> vx, vy, vz;
    std::vector<double> speed;
    std::vector<double> vux, vuy, vuz;
    std::vector<double> vunit;

    std::vector<double> ax, ay, az;
    std::vector<double> accel_mag;
    std::vector<double> aux, auy, auz;
    std::vector<double> aunit;
    std::vector<double> c_mag;

    std::vector<double> t_values;

    // Generate t values
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        t_values.push_back(t);
    }

    // Calculate velocities and accelerations
    std::vector<Point3D> velocities;
    std::vector<Point3D> accelerations;

    for (double t : t_values) {
        Point3D v = first_derivative(p0, b0, b1, p1, t);
        Point3D a = second_derivative(p0, b0, b1, p1, t);
        velocities.push_back(v);
        accelerations.push_back(a);
    }

    std::vector<double> lengths = {0.0};
    double total_length = 0.0;

    // Compute speed, acceleration magnitude, and curvature
    for (int i = 0; i < num_points; ++i) {
        double spd = get_mag(velocities[i].x, velocities[i].y, velocities[i].z);
        speed.push_back(spd);

        double acc_mag = get_mag(accelerations[i].x, accelerations[i].y, accelerations[i].z);
        accel_mag.push_back(acc_mag);

        if (i != 0) {
            double segment = std::sqrt(
                std::pow(x[i] - x[i - 1], 2) +
                std::pow(y[i] - y[i - 1], 2) +
                std::pow(z[i] - z[i - 1], 2)
            );
            lengths.push_back(segment);
            total_length += segment;

            double vux_i = velocities[i - 1].x / speed[i - 1];
            double vuy_i = velocities[i - 1].y / speed[i - 1];
            double vuz_i = velocities[i - 1].z / speed[i - 1];
            vux.push_back(vux_i);
            vuy.push_back(vuy_i);
            vuz.push_back(vuz_i);
            vunit.push_back(std::sqrt(vux_i * vux_i + vuy_i * vuy_i + vuz_i * vuz_i));

            double aux_i = accelerations[i - 1].x / accel_mag[i - 1];
            double auy_i = accelerations[i - 1].y / accel_mag[i - 1];
            double auz_i = accelerations[i - 1].z / accel_mag[i - 1];
            aux.push_back(aux_i);
            auy.push_back(auy_i);
            auz.push_back(auz_i);
            aunit.push_back(std::sqrt(aux_i * aux_i + auy_i * auy_i + auz_i * auz_i));

            double curvature = (vux_i * aux_i + vuy_i * auy_i + vuz_i * auz_i) / std::pow(speed[i - 1], 3);
            c_mag.push_back(curvature);
        }
    }

    // Add the last unit vector for velocity and acceleration
    vux.push_back(vux.back());
    vuy.push_back(vuy.back());
    vuz.push_back(vuz.back());
    aux.push_back(aux.back());
    auy.push_back(auy.back());
    auz.push_back(auz.back());
    c_mag.push_back((vux.back() * aux.back() + vuy.back() * auy.back() + vuz.back() * auz.back()) / std::pow(speed.back(), 3));

    // Get maximum curvature
    double max_curve, c_idx;
    std::tie(max_curve, c_idx) = get_max_curve(c_mag);

    // Output the results
    std::cout << "Start point:            (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << std::endl;
    std::cout << "Control point 1:        (" << b0.x << ", " << b0.y << ", " << b0.z << ")" << std::endl;
    std::cout << "Control point 2:        (" << b1.x << ", " << b1.y << ", " << b1.z << ")" << std::endl;
    std::cout << "End point:              (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
    if (new_point) {
        std::cout << "End point moved from    (" << p1_og.x << ", " << p1_og.y << ", " << p1_og.z << ") to (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
    }
    std::cout << "Maximum curvature is " << max_curve << ", reached at step " << c_idx << std::endl;

    return 0;
}
