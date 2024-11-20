"""
Algorithm for coming up with which points on the path to target.
Creates an array of points using a combination of a hermite spline and bezier curve, then calculates
the tangent vector between each of these points. These target tangent vectors are then stored
in a dictionary called targetTangentVectorsDict, with keys being points stored as tuples and values being the 
unit tangent vector at that point stored as a 1x3 np array. To obtain a point which we want to target, we call
the getTarget function. This checks the validity of target points along the path, and returns the next valid target point.
Points are considered valid if the angle between our current position vector relative to that point and the tangent
vector at that point do not exceed a predetermined angle (MAX_ANGLE). If no points along the path are valid, we create a new path
with a higher end point using our current position as a start point. This cycle repeats until we are a predetermined 
distance away from the endpoint.
"""
from HermiteSpline import createPoints, createTargetTangentVectors, MAX_ANGLE
import numpy as np

start_point = (7, 4, 0)
initial_velo = (3, 5, 12)
end_point = (0, 0, 45)
num_points = 100
END_DISTANCE = 2 # distance between current position and end point when we want to stop running

type Velocity = np.array[1,3]

x, y, z = createPoints(start_point, initial_velo, end_point, num_points)
targetTangentVectorsDict = createTargetTangentVectors(x,y,z) # dictionary where points are keys and values are the unit tangent vector at that point

def getPositionalTangentVector(current_pos: tuple, current_target: tuple) -> Velocity:
    current_pos = np.float64(current_pos)
    current_target = np.float64(current_target)
    v = np.array([current_target[0] - current_pos[0], current_target[1] - current_pos[1], current_target[2] - current_pos[2]])
    v = v/np.linalg.norm(v)
    return v

def theta(current_pos: tuple, current_target: tuple): # Angle of "error"
    v1 = getPositionalTangentVector(current_pos, current_target)
    v2 = targetTangentVectorsDict[current_target]

    dot_product = np.dot(v1, v2)
    mag_v1 = np.linalg.norm(v1)
    mag_v2 = np.linalg.norm(v2)

    cos_theta = dot_product / (mag_v1 * mag_v2)
    angle_rad = np.arccos(np.clip(cos_theta, -1, 1))
    angle_deg = np.degrees(angle_rad)
    return angle_deg

def getTarget(current_pos: tuple, current_target_iteration: int, max_angle: float, x, y, z, safety): # Checks if current target point is valid, if not returns a new point that is valid
    target = (x[current_target_iteration], y[current_target_iteration], z[current_target_iteration])
    entered_safety = safety
    distance_to_endpoint = np.linalg.norm(current_pos) - END_DISTANCE

    if distance_to_endpoint <= END_DISTANCE:
        entered_safety = True

    if entered_safety and distance_to_endpoint > END_DISTANCE and theta(current_pos, (x[len(x) - 1], y[len(y) - 1], z[len(z) - 1])) > MAX_ANGLE:
        print("Safety zone has been left and no valid points exist... Creating a new path...")
        x, y, z = createPoints(current_pos, current_velo, end_point, 100 - current_target_iteration + 10)
        end_point = (x[len(x) - 1], y[len(y) - 1], z[len(z) - 1])
        entered_safety = False
        target = getTarget(current_pos, current_velo, end_point, max_angle, x, y, z, entered_safety)
        return target

    if (current_target_iteration+1 > len(x)-1): # runs if no more valid target points
        print("No valid targets... Creating a new path...")

        ## CREATE NEW PATH
        current_pos = current_pos # Where we are now
        current_velo = (3, 5, 12) # Arbitrary velocity representing current velocity when we want to create a new path. Will probably need to create a functioning using sentence data to calculate this.
        end_point = (0, 0, 45) # Arbitrary point representing where we want to end up (can be changed if needed)

        x, y, z = createPoints(current_pos, current_velo, end_point, 100)

        ## CALL THIS FUNCTION ON SAME PATH
        target = getTarget(current_pos, current_target_iteration, max_angle, x, y, z, entered_safety)

    elif (theta(current_pos, target) > max_angle):
        target = getTarget(current_pos, current_target_iteration + 1, max_angle, x, y, z, entered_safety)
    
    return target
    
# Theta will never be 0 unless there are three target points which are collinear (unlikely)
# The sharper the turn between target points, the larger the theta

if __name__ == '__main__':
    current_pos = start_point
    target_index = 2
    max_angle = 4
    print(theta(current_pos, (x[target_index],y[target_index],z[target_index])))
    print(getTarget(current_pos, target_index, max_angle, x, y, z, False))
    print((x[target_index],y[target_index],z[target_index]))