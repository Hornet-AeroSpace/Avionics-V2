import numpy as np

# Sample data: replace with your IMU data
data = np.random.normal(0, 1, 1000)  # Simulated IMU data

# Exponential Moving Average settings
alpha = 0.1  # Smoothing factor. Smaller alpha results in more smoothing
filtered_data = np.zeros_like(data)
filtered_data[0] = data[0]  # Initialize the first element

for i in range(1, len(data)):
    filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]

