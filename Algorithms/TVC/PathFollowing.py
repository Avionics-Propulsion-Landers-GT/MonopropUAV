# Algorithm for coming up with which points on the path to target
from HermiteSpline import createPoints, createTargetTangentVectors
import numpy as np

start_point = (7, 4, 0)
initial_velo = (3, 5, 12)
end_point = (0, 0, 45)

type Velocity = np.array[1,3]

x, y, z = createPoints(start_point, initial_velo, end_point)
targetTangentVectorsDict = createTargetTangentVectors(x,y,z)

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

def getTarget(current_pos: tuple, current_target_iteration: int,max_angle): # Checks if current target point is valid, if not returns a new point that is valid
    target = (x[current_target_iteration], y[current_target_iteration], z[current_target_iteration])


    if (current_target_iteration+1 > len(x)-1):
        print("No valid targets")
       
        return None
    elif (theta(current_pos, target) > max_angle):
        target = getTarget(current_pos, current_target_iteration + 1, max_angle)
    
    
    return target
    
# Theta will never be 0 unless there are three target points which are collinear (unlikely)
# The sharper the turn between target points, the larger the theta

if __name__ == '__main__':
    current_pos = start_point
    target_index = 2
    max_angle = 4
    print(theta(current_pos, (x[target_index],y[target_index],z[target_index])))
    print(getTarget(current_pos, target_index, max_angle))
    print((x[target_index],y[target_index],z[target_index]))


