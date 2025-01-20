import numpy as np
import matplotlib.pyplot as plt
from numpy import pi


def _cross_sect_area_calc(diameter: float) -> float:
    """Calcualte the cross sectional area

    Args:
        diameter (float): Diameter of Rocket - meters

    Returns:
        float: Cross sectional aera - meters^2
    """        
    return pi * (diameter / 2) ** 2

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

print(F, G)