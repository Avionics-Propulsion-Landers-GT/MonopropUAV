#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <utility>

// Constants
const double max_tilt = 30.0;          // Maximum angle the rocket can tilt (degrees)
const double max_tvc = 15.0;           // Maximum angle achievable by Thrust Vector Control (degrees)
const double max_speed = 22.5;         // Maximum reachable speed at any arbitrary point
const double max_acceleration = 2.25;  // Maximum reachable acceleration at any arbitrary point
const double MAX_ANGLE = 13.5;         // Maximum allowable angle between positions

// Struct to represent a 3D point or vector
struct Point3D {
    double x;
    double y;
    double z;
};

// Function to calculate the Bezier curve points
std::vector<Point3D> bezier_curve(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, const std::vector<double>& t_values) {
    std::vector<Point3D> points;
    for (double t : t_values) {
        double one_minus_t = 1 - t;
        double one_minus_t_sq = one_minus_t * one_minus_t;
        double one_minus_t_cu = one_minus_t_sq * one_minus_t;
        double t_sq = t * t;
        double t_cu = t_sq * t;

        Point3D point;
        point.x = one_minus_t_cu * p0.x + 3 * one_minus_t_sq * t * p1.x + 3 * one_minus_t * t_sq * p2.x + t_cu * p3.x;
        point.y = one_minus_t_cu * p0.y + 3 * one_minus_t_sq * t * p1.y + 3 * one_minus_t * t_sq * p2.y + t_cu * p3.y;
        point.z = one_minus_t_cu * p0.z + 3 * one_minus_t_sq * t * p1.z + 3 * one_minus_t * t_sq * p2.z + t_cu * p3.z;

        points.push_back(point);
    }
    return points;
}

// Function to calculate the first derivative (velocity)
Point3D first_derivative(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t) {
    double t_sq = t * t;

    double coef0 = (-3 * t_sq) + (6 * t) - 3;
    double coef1 = (9 * t_sq) - (12 * t) + 3;
    double coef2 = (-9 * t_sq) + (6 * t);
    double coef3 = (3 * t_sq);

    Point3D derivative;
    derivative.x = coef0 * p0.x + coef1 * p1.x + coef2 * p2.x + coef3 * p3.x;
    derivative.y = coef0 * p0.y + coef1 * p1.y + coef2 * p2.y + coef3 * p3.y;
    derivative.z = coef0 * p0.z + coef1 * p1.z + coef2 * p2.z + coef3 * p3.z;

    return derivative;
}

// Function to calculate the second derivative (acceleration)
Point3D second_derivative(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t) {
    double coef0 = (-6 * t) + 6;
    double coef1 = (18 * t) - 12;
    double coef2 = (-18 * t) + 6;
    double coef3 = 6 * t;

    Point3D derivative;
    derivative.x = coef0 * p0.x + coef1 * p1.x + coef2 * p2.x + coef3 * p3.x;
    derivative.y = coef0 * p0.y + coef1 * p1.y + coef2 * p2.y + coef3 * p3.y;
    derivative.z = coef0 * p0.z + coef1 * p1.z + coef2 * p2.z + coef3 * p3.z;

    return derivative;
}

// Function to check if the angle between two points is within the allowable limit
bool angle_check(const Point3D& p0, const Point3D& p1) {
    double adj = std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));
    double opp = p1.z - p0.z;
    double angle = std::atan2(opp, adj) * 180.0 / M_PI;
    return angle >= (90.0 - MAX_ANGLE);
}

// Function to adjust the endpoint if it's not within the allowable angle
Point3D get_new_endpoint(const Point3D& p0) {
    double adj = std::sqrt(p0.x * p0.x + p0.y * p0.y);
    double angle_rad = (90.0 - MAX_ANGLE) * M_PI / 180.0;
    double z = adj * std::tan(angle_rad);
    Point3D p1 = {0.0, 0.0, z};
    return p1;
}

// Function to calculate the magnitude of a vector
double get_mag(const Point3D& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// Function to find the maximum curvature and its index
std::pair<double, double> get_max_curve(const std::vector<double>& c) {
    double max_val = c[0];
    int idx = 0;
    for (size_t i = 1; i < c.size(); ++i) {
        if (c[i] > max_val) {
            max_val = c[i];
            idx = i;
        }
    }
    double t_value = static_cast<double>(idx) / static_cast<double>(c.size() - 1);
    return std::make_pair(max_val, t_value);
}

// Function to create Bezier curve points and adjust the endpoint if necessary
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> createPoints(const Point3D& p0, const Point3D& v0, Point3D& p1, int num_points, bool& new_point) {
    Point3D p1_og = p1;
    new_point = false;

    if (!angle_check(p0, p1)) {
        p1 = get_new_endpoint(p0);
        new_point = true;
    }

    // Calculate the 2 Bezier control points using the initial conditions and endpoint
    Point3D b0 = {p0.x + (v0.x / 3.0), p0.y + (v0.y / 3.0), p0.z + (v0.z / 3.0)};
    Point3D b1 = {0.0, 0.0, b0.z + ((p1.z - p0.z) / 3.0)};

    // Create t values
    std::vector<double> t_values;
    for (int i = 0; i < num_points; ++i) {
        t_values.push_back(static_cast<double>(i) / (num_points - 1));
    }

    // Generate Bezier curve points
    std::vector<Point3D> points = bezier_curve(p0, b0, b1, p1, t_values);

    // Extract x, y, z components
    std::vector<double> x, y, z;
    for (const auto& pt : points) {
        x.push_back(pt.x);
        y.push_back(pt.y);
        z.push_back(pt.z);
    }

    return std::make_tuple(x, y, z);
}

int main() {
    // Setup initial values
    Point3D p0 = {7.0, 4.0, 0.0};      // Starting point
    Point3D v0 = {3.0, 5.0, 12.0};     // Initial velocity vector
    Point3D p1 = {0.0, 0.0, 25.0};     // Desired endpoint
    Point3D p1_og = p1;

    bool new_point = false;

    // Generate points along the Bezier curve
    int num_points = 100;
    std::vector<double> x, y, z;
    std::tie(x, y, z) = createPoints(p0, v0, p1, num_points, new_point);

    // Calculate the 2 Bezier control points using the initial conditions and endpoint
    Point3D b0 = {p0.x + (v0.x / 3.0), p0.y + (v0.y / 3.0), p0.z + (v0.z / 3.0)};
    Point3D b1 = {0.0, 0.0, b0.z + ((p1.z - p0.z) / 3.0)};

    // Initialize vectors for velocities, accelerations, etc.
    std::vector<double> vx, vy, vz;
    std::vector<double> speed;
    std::vector<double> ax, ay, az;
    std::vector<double> accel_mag;
    std::vector<double> c_mag;

    std::vector<double> t_values;
    for (int i = 0; i < num_points; ++i) {
        t_values.push_back(static_cast<double>(i) / (num_points - 1));
    }

    // Generate velocities and accelerations
    std::vector<Point3D> velocities;
    std::vector<Point3D> accelerations;
    for (double t : t_values) {
        velocities.push_back(first_derivative(p0, b0, b1, p1, t));
        accelerations.push_back(second_derivative(p0, b0, b1, p1, t));
    }

    // Compute speed and acceleration magnitude
    for (size_t i = 0; i < velocities.size(); ++i) {
        speed.push_back(get_mag(velocities[i]));
        accel_mag.push_back(get_mag(accelerations[i]));
    }

    // Compute curvature
    for (size_t i = 1; i < num_points; ++i) {
        double vx_i = velocities[i - 1].x;
        double vy_i = velocities[i - 1].y;
        double vz_i = velocities[i - 1].z;

        double ax_i = accelerations[i - 1].x;
        double ay_i = accelerations[i - 1].y;
        double az_i = accelerations[i - 1].z;

        // Cross product components
        double cross_x = vy_i * az_i - vz_i * ay_i;
        double cross_y = vz_i * ax_i - vx_i * az_i;
        double cross_z = vx_i * ay_i - vy_i * ax_i;

        double cross_mag = std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
        double curvature = cross_mag / std::pow(speed[i - 1], 3);

        c_mag.push_back(curvature);

        // Store velocities and accelerations
        vx.push_back(vx_i);
        vy.push_back(vy_i);
        vz.push_back(vz_i);

        ax.push_back(ax_i);
        ay.push_back(ay_i);
        az.push_back(az_i);
    }

    // Handle the first point where curvature cannot be computed
    c_mag.insert(c_mag.begin(), 0.0);
    vx.insert(vx.begin(), velocities[0].x);
    vy.insert(vy.begin(), velocities[0].y);
    vz.insert(vz.begin(), velocities[0].z);
    ax.insert(ax.begin(), accelerations[0].x);
    ay.insert(ay.begin(), accelerations[0].y);
    az.insert(az.begin(), accelerations[0].z);

    // Find maximum curvature
    double max_curve, c_idx;
    std::tie(max_curve, c_idx) = get_max_curve(c_mag);

    // Print results
    std::cout << "Start point:            (" << p0.x << ", " << p0.y << ", " << p0.z << ")" << std::endl;
    std::cout << "Control point 1:        (" << b0.x << ", " << b0.y << ", " << b0.z << ")" << std::endl;
    std::cout << "Control point 2:        (" << b1.x << ", " << b1.y << ", " << b1.z << ")" << std::endl;
    std::cout << "End point:              (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
    if (new_point) {
        std::cout << "End point moved from    (" << p1_og.x << ", " << p1_og.y << ", " << p1_og.z << ") to (" << p1.x << ", " << p1.y << ", " << p1.z << ")" << std::endl;
    }
    std::cout << "Maximum curvature is " << max_curve << ", reached at t = " << c_idx << std::endl;

    return 0;
}
