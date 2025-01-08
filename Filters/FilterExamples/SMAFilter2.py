import numpy as np
import matplotlib.pyplot as plt

# Sample data: replace with your IMU data
data = np.random.normal(0, 1, 1000)  # Simulated IMU data
filterdata = np.zeros(1000)
print(len(filterdata))

# SMA settings
window_size = 50  # Number of points to average

# SMA implementation using NumPy's convolution function for efficiency
sma_filtered_data = np.convolve(data, np.ones(window_size)/window_size, mode='valid')

filterdata[0:49] = data[0:49]
filterdata[49:1000] = sma_filtered_data


plt.plot(np.arange(1,len(data)+1,1), data)
plt.plot(np.arange(1,len(filterdata)+1,1), filterdata)
plt.show()
