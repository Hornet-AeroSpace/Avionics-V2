
gravity = 9.81 # m/s**2
air_density_0 = 1.225 # kg/m^3 at sea level
drag_coefficient = 0.4
cross_sect_area = cross_sect_area_calc(0.155) # m^2
mass_intital = 30000/1000 # kg


# Predict next value
def predict(u = 0.0):
    return A * x + B * u

# Return uncertainty of filter
def uncertainty():
    return A * cov * A + R

# Return the last filtered measurement
def lastMeasurement():
    return x


# Set measurement noise Q
def setMeasurementNoise(noise):
    Q = noise


# Set the process noise R
def setProcessNoise(noise):
    R = noise