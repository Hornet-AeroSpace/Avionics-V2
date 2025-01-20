import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, exp, sign


def cross_sect_area_calc(diameter: float) -> float:
    return pi * (diameter / 2) ** 2

def mass(mass_inital):
    # place holder for change in mass
    return mass_inital

def air_density(altitude: float) -> float:
    beta = 0.1354/1000.0 # Density Constant - confrim Constant
    density_sealevel = 1.225 # kg/m^3 at sea level
    density = density_sealevel * exp(-beta*altitude)
    return density

def F_aero_drag(drag_coefficient: float, cross_sect_area: float, altitude: float, velocity: float ) -> float:
    density = air_density(altitude)
    f_aero_drag = drag_coefficient* 0.5 * density * cross_sect_area * velocity**2 * sign(velocity)
    return f_aero_drag


def new_velocity(velocity: float, time_step: float, drag_coefficient: float, diameter: float, altitude: float)-> float:
    GRAVITY = 9.81 
    new_velocity = velocity - (GRAVITY * time_step) - F_aero_drag(drag_coefficient, cross_sect_area_calc(diameter), altitude, velocity)
    return










gravity = 9.81 # m/s**2
air_density_0 = 1.225 # kg/m^3 at sea level
drag_coefficient = 0.4
cross_sect_area = cross_sect_area_calc(0.155) # m^2
mass_intital = 30000/1000 # kg


#alpha = F_aero_drag(drag_coefficient, cross_sect_area,)

F = np.array([
    [1, 1],
    [0, (-gravity)]
])

G = np.array([
    [0],
    [-gravity]
])

print(F)
print(G)