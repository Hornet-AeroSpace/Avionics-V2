import numpy as np
import matplotlib.pyplot as plt
from os import path

from Filters.state_model import state_model 
from Filters.kalman_filter import kalman_filter
from Filters.imu import imu

time_start = 0
time_end = 10

# preallocating list sizes
time = np.linspace(time_start, time_end, 100)

altitude = np.zeros(100)
acceleration = np.zeros(100)
velocity = np.zeros(100)
# Simulated data varriables
noise = np.random.uniform(0.85, 1.15, 100)
acceleration_sensed = np.zeros(100)
velocity_sensed = np.zeros(100)
altitude_sensed = np.zeros(100)

# Initial conditions
altitude [0] = 0
velocity[0] = 50
velocity_sensed[0] = 50
acceleration[0] = -9.81


# Kalman Filter setup
altitude_corrected = np.zeros(100)
velocity_corrected = np.zeros(100)
varriance = np.zeros(100)
varriance_new = np.zeros(100)
velocity_corrected[0] = 40
Bno055 = imu(150E-6, 1/(time[1] - time[0])) # sample rate in Hz


for i in range(1, 100):
    print(i)
    # Simulated flight
    acceleration[i] = -9.81
    velocity[i] = velocity[i - 1] + (acceleration[i - 1] * (time[i] - time[i - 1]))
    altitude[i] = altitude[i -1] + (velocity[i - 1] * (time[i]) - time[i - 1])
    #print(altitude[i], velocity[i], acceleration[i])

    # Simulated data
    acceleration_sensed[i] = -9.81 * noise[i - 1]
    velocity_sensed[i] = velocity_sensed[i - 1] + (acceleration_sensed[i - 1] * (time[i] - time[i - 1]))
    altitude_sensed[i] = altitude_sensed[i -1] + (velocity_sensed[i - 1] * (time[i]) - time[i - 1])

    # Putting simulated data through the fitler
    Bno055.get_sensor_reading(acceleration[i])
    velocity_corrected[i], varriance[i] = kalman_filter(altitude_sensed[i], 
                                                        velocity_corrected[i-1], 
                                                        varriance[i-1], 
                                                        Bno055, 
                                                        (time[i]) - time[i - 1])
    altitude_corrected[i] = altitude_corrected[i -1] + (velocity_corrected[i - 1] * (time[i]) - time[i - 1])


fig1, ax1 = plt.subplots()
ax1.plot(time, velocity)
ax1.plot(time, velocity_sensed)
ax1.plot_date

fig2, ax2 = plt.subplots()
ax2.plot(time, altitude)
ax2.plot(time, altitude_sensed)

#plt.show()