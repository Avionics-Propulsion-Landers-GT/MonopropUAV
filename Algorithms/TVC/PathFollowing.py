# Algorithm for coming up with which points on the path to target
from HermiteSpline import createPoints
import numpy as np

start_point = (7, 4, 0)
initial_velo = (3, 5, 12)
end_point = (0, 0, 45)

x, y, z = createPoints(start_point, initial_velo, end_point)
type ArrayOfPoints = np.array
type ArrayOfVelocities = np.array
type Velocity = np.array[1,3]

def createTargetTangentVectors(x_arr: ArrayOfPoints, y_arr: ArrayOfPoints, z_arr: ArrayOfPoints) -> dict: # Creates the array of tangent vectors between any the i'th target point and the i+1 target point
    velocities = np.empty((len(x_arr),3))
    velocities = {} # dictionary where points are keys and values are the unit tangent vector at that point
    for i in range(len(x_arr)-1):
        v = np.array([x_arr[i+1] - x_arr[i], y_arr[i+1] - y_arr[i], z_arr[i+1] - z_arr[i]])
        point = (x_arr[i], y_arr[i], z_arr[i])
        # velocities[i] = v/np.linalg.norm(v)
        velocities[point] = v/np.linalg.norm(v) 
        
    return velocities

def getPositionalTangentVector(current_pos: tuple, current_target: tuple) -> Velocity:
    current_pos = np.float64(current_pos)
    current_target = np.float64(current_target)
    v = np.array([current_target[0] - current_pos[0], current_target[1] - current_pos[1], current_target[2] - current_pos[2]])
    v = v/np.linalg.norm(v)
    return v

def theta(current_pos: tuple, current_target: tuple, targetTangentVectorsDict: dict): # Angle of "error"
    v1 = getPositionalTangentVector(current_pos, current_target)
    v2 = targetTangentVectorsDict[current_target]

    dot_product = np.dot(v1, v2)
    mag_v1 = np.linalg.norm(v1)
    mag_v2 = np.linalg.norm(v2)

    cos_theta = dot_product / (mag_v1 * mag_v2)
    angle_rad = np.arccos(np.clip(cos_theta, -1, 1))
    angle_deg = np.degrees(angle_rad)
    return angle_deg


# Theta will never be 0 unless there are three target points which are collinear (unlikely)
# The sharper the turn between target points, the larger the theta

targetTangentVectorsDict = createTargetTangentVectors(x,y,z)
print(targetTangentVectorsDict[(x[0],y[0],z[0])])
print(getPositionalTangentVector(start_point, (x[1],y[1],z[1])))
print(theta((x[1],y[1],z[1]), (x[2],y[2],z[2]), targetTangentVectorsDict))



