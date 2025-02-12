from vpython import vector
import math

class AttitudeControlModel:
    def __init__(self, config):
        self.config = config
        self.control_mode = config["subsystems"]["adcs"]["control_mode"]

    def update_attitude(self, cubesat, dt):
        """Update satellite orientation based on control mode."""
        if self.control_mode == "velocity_aligned":
            self._align_with_velocity(cubesat)
        elif self.control_mode == "nadir_pointing":
            self._nadir_pointing(cubesat)
        elif self.control_mode == "sun_pointing":
            self._sun_pointing(cubesat)
        # Add more control modes as needed

    def _align_with_velocity(self, cubesat):
        """Align the satellite with its velocity vector."""
        if cubesat.velocity.mag > 0:
            velocity_direction = cubesat.velocity.norm()
            rotation_axis = cubesat.axis.cross(velocity_direction).norm()
            angle = math.acos(cubesat.axis.dot(velocity_direction))
            
            if angle != 0:
                cubesat.rotate(angle=angle, axis=rotation_axis)

    def _nadir_pointing(self, cubesat):
        """Point the satellite towards Earth's center."""
        nadir_direction = -cubesat.pos.norm()
        rotation_axis = cubesat.axis.cross(nadir_direction).norm()
        angle = math.acos(cubesat.axis.dot(nadir_direction))
        
        if angle != 0:
            cubesat.rotate(angle=angle, axis=rotation_axis)

    def _sun_pointing(self, cubesat):
        """Point the satellite towards the Sun (simplified)."""
        # TODO: Implement proper Sun position calculation
        pass 