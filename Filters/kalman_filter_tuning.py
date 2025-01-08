


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