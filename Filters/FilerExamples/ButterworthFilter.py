from scipy.signal import butter, filtfilt
import numpy as np

# Sample data: replace with your IMU data
data = np.random.normal(0, 1, 1000)  # Simulated IMU data

# Butterworth Filter settings
cutoff = 0.1  # Cutoff frequency as a fraction of the sampling rate
order = 5  # Order of the filter

# Function to design a Butterworth low-pass filter and apply it
def butter_lowpass_filter(data, cutoff, order, fs=1.0):
    nyq = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

# Apply filter
butter_filtered_data = butter_lowpass_filter(data, cutoff, order)

