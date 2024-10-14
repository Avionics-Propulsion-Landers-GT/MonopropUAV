# Given two points, return two more points that create a valid bezier curve
# Tangent line cannot exceed a certain angle relative to vertical
# Curvature cannot exceed a certain value

import matplotlib.pyplot as plt
import numpy as np
import random

max_tilt = 30 # maximum angle that rocket could tilt
max_curvature = 15 # maximum curvature (based on TVC restrictions and velocity)
const = 0.1 # arbitrary constraint value, change at will.

"""
Formula for a cubic bezier curve.

:param p0, p1, p2, p3: tuples of form (x,y,z) 
:param t: decimal between 0 and 1 representing time
:return: tuple containing the coordinates in (x,y,z) of the curve at the respective t value
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
def doesNotExceedTilt(max_tilt, crnt_orientation):
    z_vec = [0,0,1] # normal vector representing the z-vector

# check if at t we exceed the maximum curvature that our vehicle can handle.
def doesNotExceedCurva(max_curv, p0, p1, p2, p3, t):
    crnt_crv = np.linalg.norm(second_derivative(p0, p1, p2, p3, t))
    if crnt_crv > max_curv:
        print("NOT a valid path at this t.")
        return False
    else:
        print("Is a valid path at this t.")
        return True

if __name__ == '__main__': # Plot bezier curve and print out the first and second derivatives
    # generate p1 and p2 randomly (dependent on p0 and p3), then check if the path is valid.
    ## if not, see what we can do to get it right (how idk)
    p0 = (0,0,0)
    p3 = (1,1,1)
    usedPoints = []
    p1 = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
    p2 = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
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
    for i in t:
        if doesNotExceedCurva(max_curvature, p0, p1, p2, p3, i) == False:
            # generate new points to try
            # this is a bit of a brute force solution but i dont really care rn
            p1_new = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
            p2_new = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
            while (np.linalg.norm(p2_new - c) <= const or np.linalg.norm(p1_new - c) <= const for c in usedPoints):
                p1_new = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
                p2_new = tuple(random.uniform(max(p0[0], p0[1], p0[2]), max(p3[0], p3[1], p3[2])) for _ in range(3))
            break

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

    test_t = 0.8
    print(f"The first derivative when t is {test_t} is {first_derivative(p0,p1,p2,p3,test_t)}")
    print(f"The second derivative when t is {test_t} is {second_derivative(p0,p1,p2,p3,test_t)}")

    doesNotExceedCurva(max_curvature, p0, p1, p2, p3, test_t)
    plt.show()