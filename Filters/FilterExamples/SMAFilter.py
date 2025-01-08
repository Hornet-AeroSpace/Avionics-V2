import numpy as np
import matplotlib.pyplot as plt

# Sample data: replace with your IMU data
data = np.random.normal(0, 1, 1000)  # Simulated IMU data

# SMA settings
window_size = 50  # Number of points to average

# SMA implementation using NumPy's convolution function for efficiency
sma_filtered_data = np.convolve(data, np.ones(window_size)/window_size, mode='valid')

plt.plot(data)
plt.plot(sma_filtered_data)
plt.show