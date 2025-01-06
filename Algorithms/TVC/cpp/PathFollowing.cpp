// NOTE: This is untested (I was not able to run this (or any of the other C++ code for some reason))

/*
Algorithm for coming up with which points on the path to target.
Creates an array of points using a combination of a hermite spline and bezier curve, then calculates
the tangent vector between each of these points. These target tangent vectors are then stored
in a dictionary called targetTangentVectorsDict, with keys being points stored as tuples and values being the 
unit tangent vector at that point stored as a 1x3 array. To obtain a point which we want to target, we call
the getTarget function. This checks the validity of target points along the path, and returns the next valid target point.
Points are considered valid if the angle between our current position vector relative to that point and the tangent
vector at that point do not exceed a predetermined angle (MAX_ANGLE). If no points along the path are valid, we create a new path
with a higher end point using our current position as a start point. This cycle repeats until we are a predetermined 
distance away from the endpoint.
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <array>
#include <map>
#include <stdexcept>

// -------------------------------------------------------------------
// Placeholder constants and forward declarations
// -------------------------------------------------------------------

// Equivalent of Python's np array for velocity type
using Velocity = std::array<double, 3>;

// For convenience, define a point in 3D space
using Point3D = std::tuple<double, double, double>;

// We'll store x, y, z in separate arrays, mirroring the Python code
// but you could also store them in a single array of Point3D if you wish.
struct XYZPoints {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

// This is a placeholder for the maximum angle allowed
// In Python, it was: from HermiteSpline import MAX_ANGLE
static const double MAX_ANGLE = 10.0;  // Example placeholder

// End distance threshold
static const double END_DISTANCE = 2.0; // same as the Python code

// Global or external placeholders
// These mimic the Python global variables end_point and so on.
Point3D start_point  = std::make_tuple(7.0, 4.0, 0.0);
Point3D initial_velo = std::make_tuple(3.0, 5.0, 12.0);
Point3D end_point    = std::make_tuple(0.0, 0.0, 45.0);
int num_points       = 100;

// This map will mimic Python's dictionary:
//   key:   (x[i], y[i], z[i])
//   value: tangent vector (unit) at that point
std::map<Point3D, Velocity> targetTangentVectorsDict;


// -------------------------------------------------------------------
// Placeholder for createPoints(...) 
// You should replace with your actual Hermite/Bezier implementation.
// Returns a struct with x, y, z each containing num_points elements.
// -------------------------------------------------------------------
XYZPoints createPoints(const Point3D& startPt,
                       const Point3D& startVelo,
                       const Point3D& endPt,
                       int numberOfPoints) 
{
    XYZPoints result;
    // Dummy fill just to illustrate
    // In practice, this is where you'd generate Hermite/Bezier points.
    for(int i = 0; i < numberOfPoints; i++) {
        double t = static_cast<double>(i) / (numberOfPoints - 1);
        // Example: Simple linear interpolation (replace with real spline code)
        double xVal = std::get<0>(startPt) + t*(std::get<0>(endPt) - std::get<0>(startPt));
        double yVal = std::get<1>(startPt) + t*(std::get<1>(endPt) - std::get<1>(startPt));
        double zVal = std::get<2>(startPt) + t*(std::get<2>(endPt) - std::get<2>(startPt));
        result.x.push_back(xVal);
        result.y.push_back(yVal);
        result.z.push_back(zVal);
    }
    return result;
}

// -------------------------------------------------------------------
// Placeholder for createTargetTangentVectors(...)
// Again, replace with your actual function that calculates tangents.
// This function is expected to populate targetTangentVectorsDict.
// -------------------------------------------------------------------
std::map<Point3D, Velocity> createTargetTangentVectors(const std::vector<double>& x,
                                                       const std::vector<double>& y,
                                                       const std::vector<double>& z)
{
    std::map<Point3D, Velocity> dict;
    for (size_t i = 0; i < x.size(); i++) {
        // Placeholder tangent vector: 
        // We'll just store an arbitrary unit vector. 
        // Your real code would compute the actual tangent here.
        double vx = 1.0;
        double vy = 0.0;
        double vz = 0.0;
        dict[std::make_tuple(x[i], y[i], z[i])] = {vx, vy, vz};
    }
    return dict;
}

// -------------------------------------------------------------------
// Utility functions to mimic np.linalg.norm, np.dot, etc.
// -------------------------------------------------------------------
double norm3D(const Velocity &v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

double dot3D(const Velocity &v1, const Velocity &v2) {
    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]);
}

// -------------------------------------------------------------------
// getPositionalTangentVector(current_pos, current_target):
//   returns the normalized vector from current_pos to current_target.
// -------------------------------------------------------------------
Velocity getPositionalTangentVector(const Point3D &current_pos, const Point3D &current_target) {
    // Convert to doubles
    double cx = std::get<0>(current_pos);
    double cy = std::get<1>(current_pos);
    double cz = std::get<2>(current_pos);

    double tx = std::get<0>(current_target);
    double ty = std::get<1>(current_target);
    double tz = std::get<2>(current_target);

    // v = current_target - current_pos
    Velocity v { tx - cx, ty - cy, tz - cz };

    // normalize
    double length = norm3D(v);
    if (length != 0.0) {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
    return v;
}

// -------------------------------------------------------------------
// theta(current_pos, current_target):
//   returns the angle (in degrees) between:
//     (1) the positional tangent vector from current_pos to current_target
//     (2) the stored tangent vector in targetTangentVectorsDict
// -------------------------------------------------------------------
double theta(const Point3D &current_pos, const Point3D &current_target) {
    // v1 = getPositionalTangentVector(...)
    Velocity v1 = getPositionalTangentVector(current_pos, current_target);

    // v2 = targetTangentVectorsDict[current_target]
    //   If it doesn't exist, you'll need to handle that. For simplicity,
    //   we assume it always exists in the dictionary.
    auto it = targetTangentVectorsDict.find(current_target);
    if (it == targetTangentVectorsDict.end()) {
        // If not found, return something or throw an error
        throw std::runtime_error("targetTangentVectorsDict missing an entry for the current_target!");
    }
    Velocity v2 = it->second;

    // dot product
    double dot_product = dot3D(v1, v2);
    double mag_v1 = norm3D(v1);
    double mag_v2 = norm3D(v2);

    if (mag_v1 == 0.0 || mag_v2 == 0.0) {
        // Prevent division by zero
        return 0.0;
    }
    double cos_theta = dot_product / (mag_v1 * mag_v2);

    // Clip to [-1, 1] to avoid numerical issues
    if (cos_theta > 1.0)  cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    // Convert to degrees
    double angle_rad = std::acos(cos_theta);
    double angle_deg = angle_rad * (180.0 / M_PI);
    return angle_deg;
}

// -------------------------------------------------------------------
// getTarget(current_pos, current_target_iteration, max_angle, x, y, z, safety)
//   Checks if current target point is valid. If not, returns a new valid point.
// -------------------------------------------------------------------
Point3D getTarget(Point3D current_pos,
                  int current_target_iteration,
                  double max_angle,
                  std::vector<double> &x,
                  std::vector<double> &y,
                  std::vector<double> &z,
                  bool safety)
{
    // Build the target from x[current_target_iteration], y[current_target_iteration], z[current_target_iteration]
    Point3D target = std::make_tuple(x[current_target_iteration],
                                     y[current_target_iteration],
                                     z[current_target_iteration]);

    // distance_to_endpoint = np.linalg.norm(current_pos) - END_DISTANCE
    // In Python code, it's norm of current_pos, not necessarily distance to end_point. 
    // We'll do the same literal approach:
    Velocity posVec { std::get<0>(current_pos), std::get<1>(current_pos), std::get<2>(current_pos) };
    double distance_to_endpoint = norm3D(posVec) - END_DISTANCE;

    bool entered_safety = safety;

    // if distance_to_endpoint <= END_DISTANCE: entered_safety = True
    if (distance_to_endpoint <= END_DISTANCE) {
        entered_safety = true;
    }

    // if entered_safety && distance_to_endpoint > END_DISTANCE 
    //    && theta(current_pos, (x[len(x)-1], y[len(y)-1], z[len(z)-1])) > MAX_ANGLE
    if (entered_safety
        && distance_to_endpoint > END_DISTANCE
        && theta(current_pos, 
                 std::make_tuple(x[x.size() - 1],
                                 y[y.size() - 1],
                                 z[z.size() - 1])) > MAX_ANGLE)
    {
        std::cout << "Safety zone has been left and no valid points exist... Creating a new path..." << std::endl;

        // createPoints(current_pos, current_velo, end_point, 100 - current_target_iteration + 10)
        // But note that 'current_velo' is not in the signature, so presumably is global.
        // We'll assume it's the same as the initial_velo or define local logic:
        Point3D current_velo = std::make_tuple(3.0, 5.0, 12.0);

        int newNumPoints = 100 - current_target_iteration + 10;
        XYZPoints newXYZ = createPoints(current_pos, current_velo, end_point, newNumPoints);

        // Overwrite x, y, z with new arrays
        x = newXYZ.x;
        y = newXYZ.y;
        z = newXYZ.z;

        // end_point = (x[len(x)-1], y[len(y)-1], z[len(z)-1])
        end_point = std::make_tuple(x[x.size() - 1], 
                                    y[y.size() - 1], 
                                    z[z.size() - 1]);

        entered_safety = false;

        // Recursively call getTarget(...) with updated arrays
        return getTarget(current_pos, 
                         current_target_iteration, 
                         max_angle, 
                         x, y, z, 
                         entered_safety);
    }

    // if (current_target_iteration+1 > len(x)-1)
    //   => no more valid target points
    if ( (current_target_iteration + 1) > static_cast<int>(x.size()) - 1 ) {
        std::cout << "No valid targets... Creating a new path..." << std::endl;

        // CREATE NEW PATH
        // current_pos = current_pos (no change)
        // current_velo = (3, 5, 12)
        // end_point   = (0, 0, 45) 
        Point3D current_velo = std::make_tuple(3.0, 5.0, 12.0);
        end_point = std::make_tuple(0.0, 0.0, 45.0);

        // x, y, z = createPoints(current_pos, current_velo, end_point, 100)
        XYZPoints newXYZ = createPoints(current_pos, current_velo, end_point, 100);
        x = newXYZ.x;
        y = newXYZ.y;
        z = newXYZ.z;

        // Call getTarget again on the new path
        return getTarget(current_pos, 
                         current_target_iteration, 
                         max_angle, 
                         x, y, z, 
                         entered_safety);
    }
    else {
        // elif (theta(current_pos, target) > max_angle)
        if (theta(current_pos, target) > max_angle) {
            // We try the next point
            return getTarget(current_pos, 
                             current_target_iteration + 1, 
                             max_angle, 
                             x, y, z, 
                             entered_safety);
        }
    }

    // If we get here, target is valid
    return target;
}

int main() 
{
    // Setup initial data, mimicking Python's global usage
    XYZPoints points = createPoints(start_point, initial_velo, end_point, num_points);

    // Populate the global dictionary of target tangent vectors
    targetTangentVectorsDict = createTargetTangentVectors(points.x, points.y, points.z);

    // We replicate the Python test calls here:
    Point3D current_pos = start_point; 
    int target_index = 2;
    double max_angle = 4.0;

    // 1) Print out the angle
    double angleDeg = theta(current_pos, 
                            std::make_tuple(points.x[target_index],
                                            points.y[target_index],
                                            points.z[target_index]));
    std::cout << angleDeg << std::endl;

    // 2) Call getTarget(...) and print the result
    Point3D resultTarget = getTarget(current_pos, 
                                     target_index, 
                                     max_angle, 
                                     points.x, 
                                     points.y, 
                                     points.z, 
                                     false);

    std::cout << "getTarget(...) returned: (" 
              << std::get<0>(resultTarget) << ", " 
              << std::get<1>(resultTarget) << ", " 
              << std::get<2>(resultTarget) << ")" << std::endl;

    // 3) Print the original point we looked at
    std::cout << "(" 
              << points.x[target_index] << ", " 
              << points.y[target_index] << ", " 
              << points.z[target_index] << ")" << std::endl;

    return 0;
}
