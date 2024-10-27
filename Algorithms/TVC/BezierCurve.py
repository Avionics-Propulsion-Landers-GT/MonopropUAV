# Given two points, return two more points that create a valid bezier curve
# Tangent line cannot exceed a certain angle relative to vertical
# Curvature cannot exceed a certain value

import matplotlib.pyplot as plt
import numpy as np
import random

max_tilt = 30 # maximum angle that rocket could tilt
max_curvature = 15 # maximum curvature (based on TVC restrictions and velocity)
const = 0.1 # arbitrary constraint value, change at will to tune algorithm.
max_angle = 15 # maximum angle that the TVC can achieve. Degrees.

"""
Formula for a cubic bezier curve.

:param p0, p1, p2, p3: tuples of form (x,y,z) 
:param t: decimal between 0 and 1 representing time
:return: tuple containing the coordinates in (x,y,z) of the curve at the respective t value
 
TO-DO: implement Nathan's method
"""
def bezier_curve(p0, p1, p2, p3, t):
    first_term = tuple(((1 - t) ** 3) * c for c in p0)
    second_term = tuple((3 * (1 - t) ** 2) * t * c for c in p1)
    third_term = tuple((3 * (1 - t) * (t ** 2)) * c for c in p2)
    fourth_term = tuple((t ** 3) * c for c in p3)
    return tuple(first_term[c] + second_term[c] + third_term[c] + fourth_term[c] for c in range(3))

def first_derivative(p0, p1, p2, p3, t):
    first_term = tuple(((-3 * (t ** 2)) + (6 * t) - 3)*c for c in p0)
    second_term = tuple(((9 * (t ** 2)) - (12 * t) + 3)*c for c in p1)
    third_term = tuple(((-9 * (t ** 2)) + (6 * t))*c for c in p2)
    fourth_term = tuple((3 * (t ** 2))*c for c in p3)
    return tuple(first_term[c] + second_term[c] + third_term[c] + fourth_term[c] for c in range(3))

def second_derivative(p0, p1, p2, p3, t):
    first_term = tuple((-6 * t + 6)*c for c in p0)
    second_term = tuple((18 * t -12)*c for c in p1)
    third_term = tuple((-18 * t + 6)*c for c in p2)
    fourth_term = tuple((6 * t)*c for c in p3)
    return tuple(first_term[c] + second_term[c] + third_term[c] + fourth_term[c] for c in range(3))

# check if at point t we exceeded the maximum tilt 
# NOTE: Do we need this check?
# We should keep this check in, we'll need it to ensure the rocket doesn't roll out - Nathan
def doesNotExceedTilt(max_tilt, crnt_orientation):
    z_vec = [0,0,1] # normal vector representing the z-vector
    crnt_norm = crnt_orientation / np.linalg.norm(crnt_orientation)
    alpha = np.arccos(np.dot(z_vec, crnt_norm))
    if alpha >= max_tilt:
        return False
    else:
        return True

# check if at t we exceed the maximum curvature that our vehicle can handle.
def doesNotExceedCurva(max_curv, p0, p1, p2, p3, t):
    crnt_crv = np.linalg.norm(second_derivative(p0, p1, p2, p3, t))
    if crnt_crv > max_curv:
        # print("NOT a valid path at this t.")
        # for debugging
        return False
    else:
        # print("Is a valid path at this t.")
        # for debugging
        return True

def genRand(p_init, p_final):
    return tuple(random.uniform(min(p_init[0], p_init[1], p_init[2]), max(p_final[0], p_final[1], p_final[2])) for _ in range(3))

def genPoints(p_init, p_final):
    if np.dot(np.linalg.norm(np.subtract(p_final - p_init)), np.array([0,0,1])) <= max_angle :
        ## planar method (2 dimensional)
    else: 
        ## hermite curve (3 dimensional)
        # return the 2 points as tuples or as np arrays.

def vectorsAligned(v_init, v_final):
    return first_derivative(p0,p1,p2,p3,0) == v_init and first_derivative(p0,p1,p2,p3,1) == v_final

if __name__ == '__main__': # Plot bezier curve and print out the first and second derivatives
    p0 = (0,0,0)
    p3 = (1,6,1) # random end point, adjust as necesary
    usedPoints = []
    p1 = genRand(p0,p3) # use genPoints func
    p2 = genRand(p0,p3) # use genPoints func
    usedPoints.append(p1)
    usedPoints.append(p2)

    ax = plt.figure().add_subplot(projection='3d')
    ax2 = plt.figure(2).add_subplot()
    t = np.linspace(0, 1, 200) # Create np array of 200 evenly spaced values between 0 and 1

    # Prepare arrays x, y, z
    x = np.array([])
    y = np.array([])
    z = np.array([])

    # Array for graphing the curvature
    curvArr = np.array([])

    # check if at any point in t we exceed the curvature constraint
    valid_path_found = False

    # initial and final velocity vectors (to ease the process of chaining together bezier curves)
    v_init = tuple([0,0,1])
    v_final = tuple([0,0,1])
    # adjust these as necessary. 

    while not valid_path_found:
        # Generate random points
        p1 = genRand(p0,p3)
        p2 = genRand(p0,p3)
        usedPoints.append(p1)
        usedPoints.append(p2)

        valid_path_found = True  # Assume valid until proven otherwise
        
        # Check curvature and tilt for each t value in the curve
        for i in t:
            crnt_orientation = first_derivative(p0, p1, p2, p3, i)
            if not doesNotExceedCurva(max_curvature, p0, p1, p2, p3, i) or not doesNotExceedTilt(max_tilt, crnt_orientation) or not vectorsAligned(v_init, v_final):
                valid_path_found = False
                break  # No need to keep checking, start over with new random points

        if not valid_path_found:
            print("Invalid path, retrying with new random points...")

    # Creating np arrays of all the points we want to plot
    for i in t:
        point = bezier_curve(p0, p1, p2, p3, i)
        x_coordinate = point[0]
        y_coordinate = point[1]
        z_coordinate = point[2]
        curv = np.linalg.norm(second_derivative(p0, p1, p2, p3, i))
    
        x = np.append(x, x_coordinate)
        y = np.append(y, y_coordinate)
        z = np.append(z, z_coordinate)

        curvArr = np.append(curvArr, curv)
    
    ax2.plot(t, curvArr, label='curvature wrt t')
    ax2.axhline(y = max_curvature, color = 'red', linestyle = '--', label = 'Max Curvature')
    ax2.legend()
    ax.plot(x, y, z, label='bezier curve')
    ax.legend()

    # test_t = 0.8
    # print(f"The first derivative when t is {test_t} is {first_derivative(p0,p1,p2,p3,test_t)}")
    # print(f"The second derivative when t is {test_t} is {second_derivative(p0,p1,p2,p3,test_t)}")

    # print(f"P1 = {p1}, P2 = {p2}")

    # doesNotExceedCurva(max_curvature, p0, p1, p2, p3, test_t)

    # print(p0)
    # print(p1)
    # print(p2)
    # print(p3)
    
    plt.show()
