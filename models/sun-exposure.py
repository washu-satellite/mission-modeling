import numpy as np
import math


class SunExposureModel:
    def __init__(self, config):
        self.G = 6.67430e-11
        #self.M_earth = 5.972e24
        self.R_earth = 6.371e6 #meters
        self.config = config

    def calculate_sun_exposure(self, position, velocity):
        pass

    def calculate_sun_vector(self, position):
        pass

    def calculate_orbital_period(self):
        """
        Calculate the initial orbital velocity based on launch conditions' altitude range.
        Uses average altitude and inclination for simulation.
        """
        # Get altitude range from launch conditions (in km)
        launch_conditions = self.config["launch_conditions"]["orbit"]
        alt_range = launch_conditions["altitude_range"]

        
        # Calculate average altitude and inclination
        avg_altitude = (alt_range["min_km"] + alt_range["max_km"]) / 2 * 1000  # Convert to meters
        
        # Calculate orbital velocity for average altitude
        r = self.R_earth + avg_altitude #total distance from center of earth to satellite
        velocity = math.sqrt(self.G * self.M_earth / r) 
    
        
       
        return 2 * math.pi * r / velocity
    
    

    def calculate_beta_angle(self):
        # β=sin^(-1)[cos(Γ)sin(Ω)sin(i)−sin(Γ)cos(ϵ)cos(Ω)sin(i)+sin(Γ)sin(ϵ)cos(i)]
        #Γ is the argument of perigee
        #Ω is the right ascension of the ascending node
        #i is the inclination
        #ϵ is the obliquity of the ecliptic
        #Γ, Ω, i, ϵ are all in radians

       incl_range = launch_conditions["inclination_deg_range"]
       avg_inclination = (incl_range["min_deg"] + incl_range["max_deg"]) / 2
       inclination_radian = math.radians(avg_inclination)
       RAAN = 360 / 365.242199  # degrees/day (right ascension of the ascending node)
       epsilon = 23.45 #degrees
       epsilon_rad = math.radians(epsilon)
       d = 9,192 #how many days since jan 1 2000 (for march 2 2025)
       #mean anomaly
       M0 = 357.5291*(math.pi/180)
       M=M0+((2*math.pi)/365.25636)*d #radians
       #normalize to 0 to 2pi
       num_rev = M//(2*math.pi) #floor divide 
       norm_M = M - (2*math.pi)*num_rev
       lamdba = norm_M+1.915*math.sin(norm_M)+0.020*math.sin(2*norm_M) #radians 

       ra = math.atan(math.sin(lambda)*math.cos(epsilon_rad)/math.cos(lambda))
       LTAN = 13 #import from the json
       omega = ra + (15*(LTAN-12)*(math.pi/180)) #omega in radians

       beta = math.asin((math.cos(ra)*math.sin(omega)*math.sin(inclination_radian))-(math.sin(ra)*math.cos(epsilon_rad)*math.cos(omega)*math.sin(inclination_radian))+(math.sin(ra)*math.sin(epsilon_rad)*math.cos(inclination_radian)))

