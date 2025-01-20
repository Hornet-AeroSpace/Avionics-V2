import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, exp, sign


def _cross_sect_area_calc(diameter: float) -> float:
   
    return pi * (diameter / 2) ** 2

def F_aero_drag(drag_coefficient: float, cross_sect_area: float, altitude: float) -> float:
    density = air_density(altitude)
    f_aero_drag = drag_coefficient* 0.5 * density * cross_sect_area
    return f_aero_drag

def air_density(altitude: float) -> float:
    beta = 0.1354/1000.0 # Density Constant - confrim Constant
    density_sealevel = 1.225 # kg/m^3 at sea level
    density = density_sealevel * exp(-beta*altitude)
    return density

gravity = 9.81 # m/s**2
air_density_0 = 1.225 # kg/m^3 at sea level
drag_coefficient = 0.4
cross_sect_area = _cross_sect_area_calc(0.155) # m^2
mass_intital = 30000/1000 # kg




F = np.array([
    [1, 1],
    [0, 1]
])

G = np.array([
    [0],
    [-gravity]
])

print(F)
print(G)