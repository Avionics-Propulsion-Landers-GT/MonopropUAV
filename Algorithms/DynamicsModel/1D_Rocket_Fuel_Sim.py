import matplotlib.pyplot as plt
import numpy as np

class PIDF:
    def __init__(self, kp, ki, kd, kf, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def update(self, goal, actual):
        error = goal - actual
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative +
                  self.kf)
        self.previous_error = error
        return output

def calculate_projected_height(position, velocity):
    return (-velocity**2)/(2 * 9.81) + position

def simulate(mass, height, controller, pos_error_margin=0.5, vel_error_margin=0.1):
    min_thrust = 2500 * 0.4
    max_thrust = 2500
    thrust_to_m_dot = 1/(9.81 * 180)
    positions = []
    fuel_masses = []
    times = []
    position = 0
    velocity = 0
    current_time = 0
    thrust = 0
    fuel_mass_used = 0
    dt = 0.001

    while (not (abs(position - height) < pos_error_margin and
            abs(velocity) < vel_error_margin)):
        thrust = np.clip(controller.update(height, calculate_projected_height(position, velocity)), min_thrust, max_thrust)
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if int(current_time) % 100 == 0:
            pass
            # print(f"Loop 1 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {calculate_projected_height(position, velocity):.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")
    for i in range(int(1/dt * 10)):
        thrust = mass * 9.81
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if (position < 0):
            position = 0
            velocity = 0
            break
        if int(current_time) % 100 == 0:
            pass
            # print(f"Loop 1 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {calculate_projected_height(position, velocity):.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")
    while (not (abs(position - 0) < pos_error_margin and
            abs(velocity) < vel_error_margin)):
        thrust = np.clip(controller.update(0, calculate_projected_height(position, velocity)), min_thrust, max_thrust)
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if (position < 0):
            position = 0
            velocity = 0
            break
        if int(current_time) % 100 == 0:
            pass
            # print(f"Loop 2 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {calculate_projected_height(position, velocity):.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")

    print("Final Time:", current_time)
    print("Final Position:", position)
    print("Final Velocity:", velocity)
    print("Final Fuel Mass Used:", fuel_mass_used)

    return positions, fuel_masses, times

if __name__ == "__main__":
    dt = 0.001
    mass = 135
    height = 50
    positions, fuel_masses, times = simulate(mass, height, PIDF(80, 0.0, 1775, mass * 9.81, dt))
    # mass = 155
    # height = 50
    # positions, fuel_masses, times = simulate(mass, height, PIDF(240, 0.0, 1775, mass * 9.81, dt))
    # mass = 185
    # height = 50
    # positions, fuel_masses, times = simulate(mass, height, PIDF(500, 0.0, 1775, mass * 9.81, dt))
    # Plot positions over time
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.plot(times, positions, label="Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Rocket Position Over Time")
    plt.legend()
    plt.grid()

    # Plot fuel masses over time
    plt.subplot(1, 2, 2)
    plt.plot(times, fuel_masses, label="Fuel Mass Used", color="orange")
    plt.xlabel("Time (s)")
    plt.ylabel("Fuel Mass Used (kg)")
    plt.title("Fuel Mass Used Over Time")
    plt.legend()
    plt.grid()

    # Show the plots
    plt.tight_layout()
    plt.show()