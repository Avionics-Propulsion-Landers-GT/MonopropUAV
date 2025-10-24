import matplotlib.pyplot as plt
import numpy as np

def calculate_projected_height(position, velocity, min_thrust, mass):
    return solve_final_position(position, velocity, 0, -9.81 + min_thrust / mass)

def solve_final_position(x0, v0, v, a):
    """
    Returns the final position x given initial position x0, initial velocity v0,
    final velocity v, and constant acceleration a. Returns None if position cannot be reached.
    """
    if a == 0:
        if v != v0:  # Cannot change velocity without acceleration
            return None
        # For zero acceleration, velocity must remain constant
        return x0
        
    # Using v² = v0² + 2a(x - x0)
    # Rearranged to solve for x
    x = x0 + (v**2 - v0**2) / (2 * a)
    
    # Verify the solution is physically possible
    if (a > 0 and v < v0) or (a < 0 and v > v0):
        return None
        
    return x


def simulate(mass, height, pos_error_margin=0.1, vel_error_margin=0.05):
    min_thrust = 2500 * 0.4
    max_thrust = 2500
    thrust_to_m_dot = 1/(9.81 * 180)
    positions = []
    fuel_masses = []
    times = []
    thrust_magnitudes = []
    position = 0
    velocity = 0
    current_time = 0
    thrust = 0
    fuel_mass_used = 0
    dt = 0.001
    prev_second = -1

    while (abs(position - height) > pos_error_margin or
            abs(velocity) > vel_error_margin):
        if calculate_projected_height(position, velocity, min_thrust, mass) is None or calculate_projected_height(position, velocity, min_thrust, mass) > height:
            thrust = min_thrust
        else:
            thrust = max_thrust
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        thrust_magnitudes.append(thrust)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if int(current_time) != prev_second:
            prev_second = int(current_time)
            proj_pos = calculate_projected_height(position, velocity, min_thrust, mass)
            if proj_pos is None:
                proj_pos = -0
            print(f"Loop 1 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {proj_pos:.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")
            pass
    for i in range(int(1/dt * 10)):
        thrust = mass * 9.81
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        thrust_magnitudes.append(thrust)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if (position < 0):
            position = 0
            velocity = 0
            break
        if int(current_time) != prev_second:
            prev_second = int(current_time)
            proj_pos = calculate_projected_height(position, velocity, min_thrust, mass)
            if proj_pos is None:
                proj_pos = -0
            print(f"Loop 2 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {proj_pos:.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")
            pass
    while (abs(position - 0) > pos_error_margin or
            abs(velocity) > vel_error_margin):
        if calculate_projected_height(position, velocity, max_thrust, mass) is None or calculate_projected_height(position, velocity, max_thrust, mass) > 0:
            thrust = min_thrust
        else:
            thrust = max_thrust
        positions.append(position)
        fuel_masses.append(fuel_mass_used)
        times.append(current_time)
        thrust_magnitudes.append(thrust)
        current_time += dt
        position += velocity * dt
        velocity += (thrust - mass * 9.81) / mass * dt
        fuel_mass_used += thrust_to_m_dot * thrust * dt
        if (position < 0):
            position = 0
            # velocity = 0
            break
        if int(current_time) != prev_second:
            prev_second = int(current_time)
            proj_pos = calculate_projected_height(position, velocity, max_thrust, mass)
            if proj_pos is None:
                proj_pos = -0
            print(f"Loop 3 Time: {current_time:.2f}s, Position: {position:.2f}m, Velocity: {velocity:.2f}m/s, Projected Position: {proj_pos:.2f}m, Thrust: {thrust:.2f}N, Fuel Used: {fuel_mass_used:.2f}kg")
            pass

    print("Final Time:", current_time)
    print("Final Position:", position)
    print("Final Velocity:", velocity)
    print("Final Fuel Mass Used:", fuel_mass_used)
    # Write thrust magnitudes to a file
    with open('thrust_profile.txt', 'w') as f:
        f.write(str(thrust_magnitudes))

    return positions, fuel_masses, times

if __name__ == "__main__":
    dt = 0.001
    mass = 135
    height = 50
    positions, fuel_masses, times = simulate(mass, height)
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