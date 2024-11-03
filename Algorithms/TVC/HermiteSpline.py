# Given the start and end points and their corresponding velocity vectors,
# create a valid hermite spline that cannot exceed a certain orientation
# angle, speed, acceleration, and curvature.

import matplotlib.pyplot as plt
import numpy as np
import math

max_tilt = 30 # maximum angle that rocket could tilt
max_tvc = 15 # maximum angle that the TVC can achieve. Degrees.
# max_curvature = # maximum curvature (based on TVC restrictions and velocity)
max_speed = 22.5 # maximum reachable speed for any aribtrary point
max_acceleration = 2.25 # maximum reachable acceleration for any arbitrary point

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

# # Calculate the path following a hermite spline given the 
# # endpoints p0 and p1 and their corresponding velocities v0 and v1
# def hermite_spline(p0, v0, p1, v1, t):
#     v1_rel = v1 + p1
#     h0 = tuple(c * (2 * (t ** 3) - 3 * (t ** 2) + 1) for c in p0)
#     h1 = tuple(c * ((t ** 3) - 2 * (t ** 2) + t) for c in v0)
#     h2 = tuple(c * (-2 * (t ** 3) + 3 * (t ** 2)) for c in p1)
#     h3 = tuple(c * ((t ** 3) - (t ** 2)) for c in v1)
#     return tuple(h0[c] + h1[c] + h2[c] + h3[c] for c in range(3))

# # Calculate the tangent velocities at each time step for the hermite spline
# def hermite_velocity(p0, v0, p1, v1, t):
#     hv0 = tuple(c * (6 * (t ** 2) - 6 * t) for c in p0)
#     hv1 = tuple(c * (3 * (t ** 2) - 4 * t + 1) for c in v0)
#     hv2 = tuple(c * (-6 * (t ** 2) + 6 * t) for c in p1)
#     hv3 = tuple(c * (3 * (t ** 2) - 2 * t) for c in v1)
#     return tuple(hv0[c] + hv1[c] + hv2[c] + hv3[c] for c in range(3))

# # Find the magnitude of speed corresponding to each velocity point
# def hermite_speed(x, y, z):
#     return math.sqrt((x ** 2) + (y ** 2) + (z ** 2))

# # Calculate the tangent accelerations at each time step for the hermite spline
# def hermite_acceleration(p0, v0, p1, v1, t):
#     ha0 = tuple(c * (12 * t - 6) for c in p0)
#     ha1 = tuple(c * (6 * t - 4) for c in v0)
#     ha2 = tuple(c * (-12 * t + 6) for c in p1)
#     ha3 = tuple(c * (6 * t - 2) for c in v1)
#     return tuple(ha0[c] + ha1[c] + ha2[c] + ha3[c] for c in range(3))

# # Find the magnitude of the acceleration corresponding to each acceleration point
# def hermite_accel_mag(x, y, z):
#     return math.sqrt((x ** 2) + (y ** 2) + (z ** 2))

def hermite_curvature(x1, y1, z1, x2, y2, z2):
    n0 = x1 * y2 - y1 * x2
    n1 = y1 * z2 - z1 * y2
    n2 = z1 * x2 - x1 * z2
    d0 = ((x1 ** 2) + (y1 ** 2)) ** (3/2)
    d1 = ((y1 ** 2) + (z1 ** 2)) ** (3/2)
    d2 = ((z1 ** 2) + (x1 ** 2)) ** (3/2)
    return n0 / d0, n1 / d1, n2 / d2

# Check the angle between the 2 points to see if the endpoint is obtainable
def angle_check(p0, p1):
    adj = math.sqrt(((p1[0] - p0[0]) ** 2) + ((p1[1] - p0[1]) ** 2))
    opp = p1[2] - p0[2]
    angle = math.degrees(math.atan(opp / adj))
    # We use 80 as the angle to give the rocket some leeway to account for v0 being in the opposite direction, 
    # so it doesn't have to fly at the maximum 15 degree tilt the whole time to reach the end
    if angle >= 80:
        return True
    return False

def get_new_endpoint(p0):
    # Because the endpoint always wants to be above the origin (0, 0, z),
    # we can calculate the "adjacent" side of a connecting triangle using 0 for x and y of p1
    adj = math.sqrt((p0[0] ** 2) + (p0[1] ** 2))
    z = adj * math.tan(math.radians(80))
    p1 = (0, 0, z)
    return p1

# Find the magnitude of a given point relative to the origin
def get_mag(x, y, z):
    return math.sqrt((x ** 2) + (y ** 2) + (z ** 2))

def get_max_curve(c):
    max = c[0]
    idx = 0
    for i in range(100):
        if c[i] > max:
            max = c[i]
            idx = i / 100
    return max, idx

# Given the initial position, velocity, and desired end point, calculate the required bezier control points that 
# that minimize curvature by using the rule of thirds for the z points. Then, print out all the points and plot the curve
if __name__ == '__main__':
    # Setup initial values and arrays to store points
    p0 = (7, 4, 0)
    v0 = (3, 5, 12)
    p1 = (0, 0, 45)
    p1_og = p1
    # To create the most optimal path, the end velocity is dependent on the initial conditions, 
    # so we lose control over it and don't end up using it
    # v1 = (0 ,0, 15)
    new_point = False

    if angle_check(p0, p1) == False:
        p1 = get_new_endpoint(p0)
        new_point = True

    # Calculate the 2 bezier control points using the initial conditions and endpoint
    b0 = (p0[0] + (v0[0] / 3), p0[1] + (v0[1] / 3), p0[2] + (v0[2] / 3))
    b1 = (0, 0, b0[2] + ((p1[2] - p0[2]) / 3))

    x = np.array([])
    y = np.array([])
    z = np.array([])

    vx = np.array([])
    vy = np.array([])
    vz = np.array([])
    speed = []
    vux = []
    vuy = []
    vuz = []
    vunit = []

    ax = np.array([])
    ay = np.array([])
    az = np.array([])
    accel_mag = []
    aux = []
    auy = []
    auz = []
    aunit = []
    c_mag = []

    t = np.linspace(0, 1, 100)

    # Create lists of 100 evenly spaced values   
    points = bezier_curve(p0, b0, b1, p1, t)
    velocities = first_derivative(p0, b0, b1, p1, t)
    accelerations = second_derivative(p0, b0, b1, p1, t)
    lengths = [0]
    total_length = 0

    # Add each corresponding list to their respective arrays 
    x = np.append(x, points[0])
    y = np.append(y, points[1])
    z = np.append(z, points[2])

    vx = np.append(vx, velocities[0])
    vy = np.append(vy, velocities[1])
    vz = np.append(vz, velocities[2])

    ax = np.append(ax, accelerations[0])
    ay = np.append(ay, accelerations[1])
    az = np.append(az, accelerations[2])

    for i in range(100):
        if i != 0:
            segment = + math.sqrt(((points[0][i] - points[0][i - 1]) ** 2) 
                + ((points[1][i] - points[1][i - 1]) ** 2) + ((points[2][i] - points[2][i - 1]) ** 2))
            lengths.append(segment)
            total_length = total_length + segment
            vux.append(vx[i - 1] / speed[i - 1])
            vuy.append(vy[i - 1] / speed[i - 1])
            vuz.append(vz[i - 1] / speed[i - 1])
            vunit.append(math.sqrt(vux[i - 1] ** 2 + vuy[i - 1] ** 2 + vuz[i - 1] ** 2))
            aux.append(ax[i - 1] / accel_mag[i - 1])
            auy.append(ay[i - 1] / accel_mag[i - 1])
            auz.append(az[i - 1] / accel_mag[i - 1])
            c_mag.append((vux[i - 1] * aux[i - 1] + vuy[i - 1] * auy[i - 1] + vuz[i - 1] * auz[i - 1]) / (speed[i - 1] ** 3))
        
        speed.append(get_mag(velocities[0][i], velocities[1][i], velocities[2][i]))
        accel_mag.append(get_mag(accelerations[0][i], accelerations[1][i], accelerations[2][i]))

    # Add the last unit vector for velocity and acceleration
    vux.append(vux[98])
    vuy.append(vuy[98])
    vuz.append(vuz[98])
    aux.append(aux[98])
    auy.append(auy[98])
    auz.append(auz[98])
    c_mag.append((vux[99] * aux[99] + vuy[99] * auy[99] + vuz[99] * auz[99]) / (speed[99] ** 3))

    max_curve, c_idx = get_max_curve(c_mag)

    # Add hermite spline with the path and all control / velocity points to a 3D graph
    path_plot = plt.axes(projection='3d')
    path_plot.scatter3D(p0[0], p0[1], p0[2], color='red')
    path_plot.scatter3D(p1[0], p1[1], p1[2], color='red')
    path_plot.scatter3D(b0[0], b0[1], b0[2], color='blue')
    path_plot.scatter3D(b1[0], b1[1], b1[2], color='blue')
    path_plot.plot(x, y, z, color='purple', label='bezier curve')
    path_plot.set_xscale('linear')
    path_plot.set_yscale('linear')
    path_plot.set_zscale('linear')
    path_plot.set_xlabel('X axis')
    path_plot.set_ylabel('Y axis')
    path_plot.set_zlabel('Z axis')
    path_plot.set_title('Hermite spline')
    path_plot.set_aspect('equal')

    # Plot the velocities and speed to a 2D graph
    velocity_plot = plt.figure(2).add_subplot()
    velocity_plot.plot(t, vx, color='red')
    velocity_plot.plot(t, vy, color='green')
    velocity_plot.plot(t, vz, color='blue')
    velocity_plot.plot(t, speed, color='purple')
    velocity_plot.set_xlabel('T step')
    velocity_plot.set_ylabel('Speed')

    # Plot the acceleration and its magnitude to a 2D graph
    acceleration_plot = plt.figure(3).add_subplot()
    acceleration_plot.plot(t, ax, color='red')
    acceleration_plot.plot(t, ay, color='green')
    acceleration_plot.plot(t, az, color='blue')
    acceleration_plot.plot(t, accel_mag, color='purple')
    acceleration_plot.set_xlabel('T step')
    acceleration_plot.set_ylabel('Acceleration')

    # Plot the curvature to a 2D graph
    curvature_plot = plt.figure(4).add_subplot()
    curvature_plot.plot(t, c_mag, color='purple')

    print(f"Start point:            {p0}")
    print(f"Control point 1:        {b0}")
    print(f"Control point 2:        {b1}")
    print(f"End point:              {p1}")
    if new_point == True:
        print(f"End point moved from    {p1_og} to {p1}")
    print(f"Maximum curvature is {max_curve}, reached at step {c_idx}")

    plt.show()
