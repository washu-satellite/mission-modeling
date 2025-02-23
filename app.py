from vpython import vector, box, sphere, canvas, color, textures, curve, rate, button, menu
import json
import time
import threading
from models.kinematics_model import KinematicsModel
from models.attitude_control import AttitudeControlModel

VISUAL_SCALE = 1 / 1000

class MissionSimulation:
    def __init__(self):
        self.load_mission_configs()
        self.setup_scene()
        self.setup_controls()
        self.stop_event = threading.Event()

    def load_mission_configs(self):
        """Load all available mission configurations."""
        # In practice, you might want to scan a directory for all .json files
        self.missions = {
            "SCALAR": "missions/SCALAR.json"
        }

    def load_mission_config(self, mission_name):
        """Load specific mission configuration."""
        with open(self.missions[mission_name], 'r') as f:
            return json.load(f)

    def setup_scene(self):
        """Setup the visualization scene."""
        # Create scene first and ensure it's fully initialized
        self.scene = canvas(title="Orbital Mechanics Simulation", 
                          width=800, height=600,
                          center=vector(0, 0, 0),
                          background=color.black)   # Allow user to pan  # Give the server time to initialize

        # Create Earth
        self.earth = sphere(pos=vector(0, 0, 0), 
                          radius=6.371e6 * VISUAL_SCALE, 
                          texture=textures.earth)  # Ensure Earth is created before rotating

        # Initialize trail
        self.trail = curve(color=color.cyan, radius=10, opacity=0.9)

    def setup_controls(self):
        """Setup browser-based controls GUI."""
        # Create a dropdown menu for mission selection
        self.mission_select = menu(
            choices=list(self.missions.keys()),
            selected="SCALAR",
            bind=self.on_mission_select
        )

        # Create start and stop buttons
        self.start_button = button(
            text="Start Simulation",
            bind=self.start_simulation
        )
        
        self.stop_button = button(
            text="Stop Simulation",
            bind=self.stop_simulation
        )

    def on_mission_select(self, selection):
        """Handle mission selection."""
        self.selected_mission = selection.selected

    def create_cubesat(self, config):
        """Create CubeSat object based on configuration."""
        cubesat_config = config["cubesat_configuration"]

        cubesat = box(pos=vector(0, 0, 0), 
                     size=vector(100, 100, 100), 
                     color=color.yellow)

        cubesat.mass = cubesat_config["mass"]
        cubesat.cross_sectional_area = cubesat_config["cross_sectional_area"]

        return cubesat

    def simulate_mission(self, cubesat, config):
        """Main simulation loop."""
        dt = config["simulation_parameters"]["time_step"]
        sim_rate = config["simulation_parameters"]["simulation_rate"]

        # Initialize models
        self.kinematics_model = KinematicsModel(config)
        self.attitude_control = AttitudeControlModel(config)

        # Set initial position and velocity
        initial_conditions = config["initial_conditions"]
        altitude = initial_conditions["orbit"]["altitude"]
        velocity = initial_conditions["orbit"]["velocity"]

        cubesat.pos = self.kinematics_model.calculate_initial_position(0, 0, altitude, VISUAL_SCALE)
        cubesat.velocity = vector(velocity["x"], velocity["y"], velocity["z"])
        cubesat.acceleration = vector(0, 0, 0)

        last_update = time.time()
        fixed_dt = 1.0 / sim_rate

        while not self.stop_event.is_set():
            # Check timing for physics calculations
            current_time = time.time()
            elapsed = current_time - last_update
            if elapsed < fixed_dt:
                continue

            # Check if the orbit has decayed
            current_alt =  (cubesat.pos.mag / VISUAL_SCALE) - self.kinematics_model.R_earth
            if current_alt < 100000:
                print("Satellite decayed.")
                break

            # Update kinematics
            self.kinematics_model.update_kinematics(cubesat, dt, VISUAL_SCALE)
            
            # Update attitude
            self.attitude_control.update_attitude(cubesat, dt)

            # Update visualization
            self.trail.append(pos=cubesat.pos)
            if self.trail.npoints > 5000:  # Limit trail length
                self.trail.pop(0)

            # Follow the cubesat with the camera
            self.scene.camera.follow(cubesat)
            
            rate(sim_rate)  # Control simulation speed

            # Update physics timing
            last_update = current_time

    def start_simulation(self, *args):
        """Start the simulation with selected mission."""
        mission_name = self.mission_select.selected
        config = self.load_mission_config(mission_name)
        
        # Clear existing trail
        self.trail.clear()
        
        # Create new CubeSat
        self.cubesat = self.create_cubesat(config)
        
        # Start simulation in a thread
        self.stop_event.clear()
        simulation_thread = threading.Thread(
            target=self.simulate_mission, 
            args=(self.cubesat, config)
        )
        simulation_thread.daemon = True
        simulation_thread.start()

    def stop_simulation(self):
        self.stop_event.set()

if __name__ == "__main__":
    sim = MissionSimulation()
    while True:
        rate(1)  # Keep the server alive