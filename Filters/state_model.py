import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, exp, sign

from .atm_properties import atmospheric_density

def cross_sect_area_calc(diameter: float) -> float:
    return pi * (diameter / 2) ** 2

def mass(mass_inital):
    # place holder for change in mass
    return mass_inital


def F_aero_drag(drag_coefficient: float, cross_sect_area: float, altitude: float, velocity: float ) -> float:
    density = atmospheric_density(altitude)
    f_aero_drag = drag_coefficient* 0.5 * density * cross_sect_area * velocity**2 * sign(velocity)
    return f_aero_drag


def state_model(altitude: float, velocity: float)-> float:
    """State model of forces/mass, returning the acceleration of the rocket

    Args:
        altitude (float): altitude (meters) for air density
        velocity (float): velocity (meters/sec) for aerodynamic drag calc
        

    Returns:
        float: acceeration of rocket
    """    
    GRAVITY = 9.81 
    DRAG_COEFFICIENT = 0.4
    CROSS_SECT_AREA = cross_sect_area_calc (0.155)
    MASS = 30 # Kg

    force_aero = F_aero_drag(DRAG_COEFFICIENT, CROSS_SECT_AREA, altitude, velocity)
    acceleration =  - (GRAVITY + (force_aero / MASS))
    return acceleration
 
