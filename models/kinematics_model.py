from vpython import vector
import math

class KinematicsModel:
    def __init__(self, config):
        self.G = 6.67430e-11
        self.M_earth = 5.972e24
        self.R_earth = 6.371e6
        self.J2 = 1.08263e-3
        self.config = config
        
        # Atmospheric parameters
        self.rho_0 = 1.225
        self.H_scale = 8500
        
    def calculate_initial_position(self, latitude, longitude, altitude, scale):
        """Calculate the initial position vector from latitude, longitude, and altitude."""
        phi = math.radians(latitude)
        theta = math.radians(longitude)
        r = self.R_earth + altitude
        x = r * math.cos(phi) * math.cos(theta)
        y = r * math.cos(phi) * math.sin(theta)
        z = r * math.sin(phi)
        return vector(x, y, z) * scale

    def calculate_forces(self, position, velocity, mass, cross_section, scale):
        """Calculate all forces acting on the satellite."""
        r = position.mag / scale
        altitude = r - self.R_earth

        # Calculate individual forces
        gravity = self._gravitational_force(position, mass, scale)
        drag = self._drag_force(position, velocity, altitude, cross_section)
        j2 = self._j2_perturbation(position, scale)

        return gravity + drag + j2

    def _gravitational_force(self, position, mass, scale):
        """Calculate gravitational force."""
        r = position.mag / scale
        return (-self.G * self.M_earth * mass / (r ** 2) * 
                position.norm())

    def _atmospheric_density(self, altitude):
        """Calculate atmospheric density at given altitude."""
        if altitude < 0:
            return self.rho_0
        return self.rho_0 * math.exp(-altitude / self.H_scale)

    def _drag_force(self, position, velocity, altitude, cross_section):
        """Calculate atmospheric drag force."""
        C_d = self.config["cubesat_configuration"].get("drag_coefficient", 2.2)
        rho = self._atmospheric_density(altitude)
        velocity_mag = velocity.mag
        
        if velocity_mag == 0:
            return vector(0, 0, 0)
            
        drag_force_mag = 0.5 * rho * velocity_mag**2 * C_d * cross_section
        return -drag_force_mag * velocity.norm()

    def _j2_perturbation(self, position, scale):
        """Calculate J2 perturbation force."""
        r = position.mag / scale
        z = position.z / scale
        
        factor = (3/2) * self.J2 * (self.R_earth ** 2) * (self.G * self.M_earth) / (r ** 4)
        
        perturb_x = factor * (position.x / r) * (5 * (z / r) ** 2 - 1)
        perturb_y = factor * (position.y / r) * (5 * (z / r) ** 2 - 1)
        perturb_z = factor * (position.z / r) * (5 * (z / r) ** 2 - 3)
        
        return vector(perturb_x, perturb_y, perturb_z) * scale

    def update_kinematics(self, cubesat, dt, scale):
        """Update position and velocity based on forces."""
        net_force = self.calculate_forces(
            cubesat.pos, 
            cubesat.velocity, 
            cubesat.mass, 
            cubesat.cross_sectional_area,
            scale
        )
        
        # Update motion
        cubesat.acceleration = net_force / cubesat.mass
        cubesat.velocity += cubesat.acceleration * dt
        cubesat.pos += cubesat.velocity * dt * scale
