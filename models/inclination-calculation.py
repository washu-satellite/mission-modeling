import numpy as np
import matplotlib.pyplot as plt
#plt.ion() #interactive on
print("Starting script...")

# Constants
OmegaDotEarth = 360 / 365.242199  # degrees/day
OmegaDotSatellite = OmegaDotEarth  # SSO condition
J2 = 0.00108263  # Earth’s oblateness
Re = 6378.137    # Earth’s radius (km)
mu = 398600.4418 # Gravitational parameter (km³/s²)

print("Constants defined.")

# Altitude vector
altitude = np.linspace(200, 1000, 50)  # km
a = Re + altitude  # Semi-major axis (km)
print(f"Altitude range: {altitude[0]} to {altitude[-1]} km")

# Mean motion (n) in radians/second
n = np.sqrt(mu / a**3)
print("Mean motion calculated.")

# Convert OmegaDot to radians/second
OmegaDot_rad_s = OmegaDotSatellite * (np.pi / 180) / (24 * 3600)
print(f"OmegaDot: {OmegaDot_rad_s:.8f} rad/s")

# Calculate inclination
try:
    cos_i = -OmegaDot_rad_s / (1.5 * n * J2 * (Re / a)**2)
    cos_i = np.clip(cos_i, -1, 1)  # Ensure valid range for arccos
    i_rad = np.arccos(cos_i)
    i_deg = np.degrees(i_rad)
    print("Inclination calculated successfully.")
except Exception as e:
    print(f"Error in calculation: {e}")
    exit()

# Plotting
try:
    plt.figure(figsize=(10, 6))
    plt.plot(altitude, i_deg, 'b-', label='Inclination for SSO')
    plt.xlabel('Altitude (km)')
    plt.ylabel('Inclination (degrees)')
    plt.title('Inclination vs. Altitude for Sun-Synchronous Orbit')
    plt.grid(True)
    plt.legend()
    print("Plot configured.")
    plt.show(block=True)
    print("Plot displayed.")
except Exception as e:
    print(f"Error in plotting: {e}")

# Sample output
for alt, inc in zip(altitude[::10], i_deg[::10]):
    print(f"Altitude: {alt:.0f} km, Inclination: {inc:.2f}°")
