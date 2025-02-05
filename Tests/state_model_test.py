import numpy as np
import matplotlib.pyplot as plt
from os import path

from Filters.state_model import state_model 

time_start = 0
time_end = 10

# preallocating list sizes
time = np.linspace(time_start, time_end, 100)
altitude = np.zeros(100)
acceleration = np.zeros(100)
velocity = np.zeros(100)

# Initial conditions
altitude [0] = 0
velocity[0] = 50
acceleration[0] = -9.81


for i in range(1, 100):
    acceleration[i] = -9.81
    velocity[i] = velocity[i - 1] + (acceleration[i - 1] * (time[i] - time[i - 1]))
    altitude[i] = altitude[i -1] + (velocity[i - 1] * (time[i]) - time[i - 1])
    #print(altitude[i], velocity[i], acceleration[i])


fig1, ax1 = plt.subplots()
ax1.plot(time, velocity)

fig2, ax2 = plt.subplots()
ax2.plot(time, altitude)


# noise simulation
noise = np.random.uniform(0.85, 1.15, 100)
acceleration_sensed = np.zeros(100)
velocity_sensed = np.zeros(100)
altitude_sensed = np.zeros(100)
velocity_sensed[0] = 50

for i in range(1, 100):

    acceleration_sensed[i] = -9.81 * noise[i - 1]
    velocity_sensed[i] = velocity_sensed[i - 1] + (acceleration_sensed[i - 1] * (time[i] - time[i - 1]))
    altitude_sensed[i] = altitude_sensed[i -1] + (velocity_sensed[i - 1] * (time[i]) - time[i - 1])


ax1.plot(time, velocity_sensed)
ax2.plot(time, altitude_sensed)
plt.show()