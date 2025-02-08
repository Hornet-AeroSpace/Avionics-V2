import numpy as np
import matplotlib.pyplot as plt
from os import path

from Filters.state_model import state_model 
from Filters.kalman_filter import kalman_filter
from Filters.imu import imu

# Time set up for Simulation
time_start = 0
time_end = 10
sample_rate = 250 # sample rate in Hz
time_step = 1 / sample_rate
samples = int((time_end - time_start) / time_step)

# preallocating list sizes
#time = np.linspace(time_start, time_end, 100)
time = np.arange(time_start, time_end, time_step)

altitude = np.zeros(samples)
acceleration = np.zeros(samples)
velocity = np.zeros(samples)
# Simulated data varriables
noise = np.random.uniform(0.85, 1.15, samples)
acceleration_sensed = np.zeros(samples)
velocity_sensed = np.zeros(samples)
altitude_sensed = np.zeros(samples)

# Initial conditions
altitude [0] = 0
velocity[0] = 50
velocity_sensed[0] = 50
acceleration[0] = -9.81


# Kalman Filter setup
altitude_corrected = np.zeros(samples)
velocity_corrected = np.zeros(samples)
varriance = np.zeros(samples)
varriance_new = np.zeros(samples)
velocity_corrected[0] = 40 # simulated first sensor reading.
Bno055 = imu(150E-6, sample_rate) # sample rate in Hz


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