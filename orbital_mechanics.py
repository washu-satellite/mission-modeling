from vpython import vector, box, sphere, canvas, color, textures, curve, rate, norm, label
import math
import tkinter as tk
from tkinter import ttk
import threading
import random

# Constants
G = 6.67430e-11  # Gravitational constant (m^3/kg/s^2)
M_earth = 5.972e24  # Mass of Earth (kg)
R_earth = 6.371e6  # Radius of Earth (m)
VISUAL_SCALE = 1 / 1000  # Scale distances for visualization
rho_0 = 1.225  # Sea-level atmospheric density (kg/m^3)
H_scale = 8500  # Scale height of atmosphere (m)
C_d = 2.2  # Drag coefficient (assumed for CubeSat)
omega_earth = 7.2921e-5  # Earth's angular velocity in rad/s

# CubeSat size configurations (1U, 2U, 3U, 6U, 12U)
CUBESAT_SIZES = {
    "1U": {"mass": 1.33, "cross_sectional_area": 0.01},
    "2U": {"mass": 2.66, "cross_sectional_area": 0.02},
    "3U": {"mass": 4.0, "cross_sectional_area": 0.03},
    "6U": {"mass": 8.0, "cross_sectional_area": 0.06},
    "12U": {"mass": 16.0, "cross_sectional_area": 0.12},
}


def calculate_position(latitude, longitude, altitude):
    """Calculate the initial position vector from latitude, longitude, and altitude."""
    phi = math.radians(latitude)  # Latitude in radians
    theta = math.radians(longitude)  # Longitude in radians

    r = R_earth + altitude
    x = r * math.cos(phi) * math.cos(theta)
    y = r * math.cos(phi) * math.sin(theta)
    z = r * math.sin(phi)

    return vector(x, y, z) * VISUAL_SCALE


def calculate_velocity(position, altitude):
    """Calculate the initial velocity vector for an eastward equatorial orbit with fixed direction."""
    r = position.mag / VISUAL_SCALE
    v_circular = math.sqrt(G * M_earth / r)  # Circular orbital velocity

    # Eastward tangential direction
    eastward = vector(-position.y, position.x, 0).norm()

    # Add Earth's rotational velocity at the surface to the orbital velocity
    rotational_velocity = vector(-omega_earth *
                                 position.y, omega_earth * position.x, 0)

    # Total velocity vector with fixed eastward direction
    velocity_vector = v_circular * eastward + rotational_velocity

    return velocity_vector


def atmospheric_density(altitude):
    """Calculate atmospheric density based on altitude using an exponential model."""
    return rho_0 * math.exp(-altitude / H_scale)


def j2_perturbation(pos):
    """Calculate J2 perturbation effects."""
    J2 = 1.08263e-3  # Earth's J2 coefficient
    r = pos.mag / VISUAL_SCALE
    z = pos.z / VISUAL_SCALE

    factor = (3 / 2) * J2 * (R_earth ** 2) * (G * M_earth) / (r ** 4)
    perturb_x = factor * (pos.x / r) * (5 * (z / r) ** 2 - 1)
    perturb_y = factor * (pos.y / r) * (5 * (z / r) ** 2 - 1)
    perturb_z = factor * (pos.z / r) * (5 * (z / r) ** 2 - 3)

    return vector(perturb_x, perturb_y, perturb_z) * VISUAL_SCALE


def simulate_orbit(cubesat, trail, scene, stop_event):
    """Simulate the CubeSat's orbital trajectory with one face always pointing towards Earth while maintaining its shape."""
    dt = 1  # Time step in seconds
    t = 0
    max_altitude = 0
    total_distance = 0
    initial_velocity = cubesat.velocity.mag

    while not stop_event.is_set():
        r = cubesat.pos.mag / VISUAL_SCALE
        altitude = r - R_earth

        # Break simulation if satellite decays too low
        if altitude < 100000:  # 100 km - approximate atmospheric boundary
            print(f"Satellite decayed. Simulation ended at {t/3600:.2f} hours")
            break

        # Track maximum altitude
        max_altitude = max(max_altitude, altitude)

        # Calculate distance traveled
        total_distance += cubesat.velocity.mag * dt

        # Advanced atmospheric density calculation
        rho = atmospheric_density(altitude)

        # Comprehensive drag force calculation
        velocity_mag = cubesat.velocity.mag
        drag_force_mag = 0.5 * rho * velocity_mag**2 * C_d * cubesat.cross_sectional_area
        drag_force = -drag_force_mag * norm(cubesat.velocity)

        # Gravitational Force
        gravity_force = -G * M_earth * \
            cubesat.mass / (r ** 2) * norm(cubesat.pos)

        # J2 Perturbation
        perturb_force = j2_perturbation(cubesat.pos)

        # Net Force
        net_force = gravity_force + drag_force + perturb_force

        # Update motion
        cubesat.acceleration = net_force / cubesat.mass
        cubesat.velocity += cubesat.acceleration * dt
        cubesat.pos += cubesat.velocity * dt * VISUAL_SCALE

        # Rotation to align box with velocity vector
        # Reset cube orientation to a known state
        cubesat.axis = vector(1, 0, 0)  # Reset to initial 'forward'
        cubesat.up = vector(0, 0, 1)  # Reset to initial 'up'

        if cubesat.velocity.mag > 0:  # Avoid rotation when at rest
            velocity_direction = norm(cubesat.velocity)
            # Define an axis of rotation perpendicular to both the current forward and the new velocity direction
            rotation_axis = norm(cubesat.axis.cross(velocity_direction))
            # Angle between current forward and velocity direction
            angle = math.acos(cubesat.axis.dot(velocity_direction))

            # Rotate the CubeSat to align with its velocity
            cubesat.rotate(angle=angle, axis=rotation_axis)

        cubesat.size = vector(100, 100, 100)

        # Update trail with fading effect
        trail.append(pos=cubesat.pos)
        if trail.npoints > 50000:  # Limit trail length
            trail.pop(0)

        scene.camera.follow(cubesat)

        t += dt
        rate(100)


def main():
    # Tkinter UI for configuration
    def start_simulation():
        cubesat_size = size_input.get()
        altitude = float(altitude_input.get()) * 1000  # Convert km to meters

        # Configure CubeSat properties
        cubesat.mass = CUBESAT_SIZES[cubesat_size]["mass"]
        cubesat.cross_sectional_area = CUBESAT_SIZES[cubesat_size]["cross_sectional_area"]

        # Calculate initial position and velocity
        # latitude = 0  # Kennedy Space Center latitude
        # longitude = 0  # Kennedy Space Center longitude
        cubesat.pos = calculate_position(
            cubesat.pos.x, cubesat.pos.y, altitude)

        # Clear trail
        trail.clear()

        # Start simulation thread
        stop_event.clear()
        simulation_thread = threading.Thread(
            target=simulate_orbit, args=(cubesat, trail, scene, stop_event))
        simulation_thread.daemon = True
        simulation_thread.start()

    def stop_simulation():
        stop_event.set()

    # Tkinter window
    root = tk.Tk()
    root.title("CubeSat Orbital Simulation")

    # Input fields
    ttk.Label(root, text="CubeSat Size:").grid(
        row=0, column=0, padx=10, pady=10)
    size_input = ttk.Combobox(root, values=list(CUBESAT_SIZES.keys()))
    size_input.grid(row=0, column=1, padx=10, pady=10)
    size_input.set("1U")

    ttk.Label(root, text="Initial Altitude (km):").grid(
        row=1, column=0, padx=10, pady=10)
    altitude_input = ttk.Entry(root)
    altitude_input.grid(row=1, column=1, padx=10, pady=10)
    altitude_input.insert(0, "2000")

    start_button = ttk.Button(
        root, text="Start Simulation", command=start_simulation)
    start_button.grid(row=3, column=0, padx=10, pady=10)

    stop_button = ttk.Button(
        root, text="Stop Simulation", command=stop_simulation)
    stop_button.grid(row=3, column=1, padx=10, pady=10)

    # VPython scene
    scene = canvas(title="Orbital Mechanics Simulation", width=1920,
                   height=1080, center=vector(0, 0, 0), background=color.black)
    earth = sphere(pos=vector(0, 0, 0), radius=R_earth *
                   VISUAL_SCALE, texture=textures.earth)

    # Rotate around x-axis for axial tilt
    earth.rotate(angle=math.radians(80.6), axis=vector(0, 1, 0))
    earth.rotate(angle=math.radians(28.6), axis=vector(1, 0, 0))

    cubesat = box(pos=vector(90, 0, 0), size=vector(
        100, 100, 100), color=color.yellow)

    cubesat.rotate(angle=math.radians(90), axis=vector(0, 1, 0))
    cubesat.mass = CUBESAT_SIZES["1U"]["mass"]
    cubesat.cross_sectional_area = CUBESAT_SIZES["1U"]["cross_sectional_area"]
    cubesat.velocity = vector(6489.95, 4326.63, 0)
    cubesat.acceleration = vector(0, 0, 0)
    trail = curve(color=color.cyan, radius=10, opacity=0.9)

    # Stop event for simulation thread
    stop_event = threading.Event()

    # Run Tkinter main loop
    root.mainloop()


if __name__ == "__main__":
    main()
