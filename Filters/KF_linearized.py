import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, exp, sign


def _cross_sect_area_calc(diameter: float) -> float:
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


def new_velocoity(velocity: float, time_step: float, drag_coefficient: float, cross_sect_area: float, altitude: float)-> float:
    GRAVITY = 9.81 
    new_velocity = velocity - (GRAVITY * time_step) - F_aero_drag(drag_coefficient, cross_sect_area, altitude, velocity)
    return


# Main differential equation 
def derivative(t: float, state: np.array, rocket: RocketConfig, motor: Motor) -> np.ndarray:
    """State space equation to be integrated numericaly. 

    Args:
        t (float): Time of current step in integratoin - seconds
        state (np.array): State Vector [altitude - m, velocity - m/s, mass -kg]
        rocket (RocketConfig): RocketConfig class containing perameters/methods of rocket
        motor (Motor): Motor class conatining peramters/methods of the motor

    Returns:
        np.ndarray: State array dervivative to be integrated [altitude_dot - m/s, velcoity_dot - m/s**2, mass_dot - kg/s]
    """    
    GRAVITY = 9.81 # m/s^2 change in gravity considered negligible
    aero = Aero(rocket)

    # State vector
    altitude = state[0]
    velocity = state[1]
    mass = state[2]
    
    # Forces
    f_gravity = GRAVITY * mass
    f_aero = aero.F_aero_drag(velocity, altitude)
    f_thrust, mass_dot = motor.motor_output(t)
    
    f_net = f_thrust - f_aero - f_gravity
    acceleration = f_net/mass
    
    state_dot = np.array([velocity, acceleration, mass_dot])
    
    return state_dot











gravity = 9.81 # m/s**2
air_density_0 = 1.225 # kg/m^3 at sea level
drag_coefficient = 0.4
cross_sect_area = _cross_sect_area_calc(0.155) # m^2
mass_intital = 30000/1000 # kg


alpha = F_aero_drag(drag_coefficient, cross_sect_area,)

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