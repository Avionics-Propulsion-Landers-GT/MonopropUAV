# Given two points, return two more points that create a valid bezier curve
# Tangent line cannot exceed a certain angle relative to vertical
# Curvature cannot exceed a certain value

import matplotlib.pyplot as plt
import numpy as np

max_tilt = 30 # maximum angle that rocket could tilt
max_curvature = 15 # maximum curvature (based on TVC restrictions and velocity)

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


if __name__ == '__main__': # Plot bezier curve and print out the first and second derivatives
    # Arbitrary points, will choose later on
    p0 = (0,0,0)
    p1 = (0.1,0.2,0.3)
    p2 = (-5,-2,-3)
    p3 = (1,1,1)

    ax = plt.figure().add_subplot(projection='3d')

    t = np.linspace(0, 1, 200) # Create np array of 200 evenly spaced values between 0 and 1

    # Prepare arrays x, y, z
    x = np.array([])
    y = np.array([])
    z = np.array([])

    # Creating np arrays of all the points we want to plot
    for i in t:
        point = bezier_curve(p0, p1, p2, p3, i)
        x_coordinate = point[0]
        y_coordinate = point[1]
        z_coordinate = point[2]

        x = np.append(x, x_coordinate)
        y = np.append(y, y_coordinate)
        z = np.append(z, z_coordinate)

    ax.plot(x, y, z, label='bezier curve')
    ax.legend()

    test_t = 0.8
    print(f"The first derivative when t is {test_t} is {first_derivative(p0,p1,p2,p3,test_t)}")
    print(f"The second derivative when t is {test_t} is {second_derivative(p0,p1,p2,p3,test_t)}")

    plt.show()



