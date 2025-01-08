from filterpy.kalman import KalmanFilter

f = KalmanFilter (dim_x=6, dim_z=3)  # Example dimensions, adjust as necessary
f.x = initial_state  # initial state (position and velocity)
f.F = transition_matrix  # state transition matrix
f.H = measurement_function  # measurement function
f.P *= 1000.  # covariance matrix
f.R = measurement_noise_cov  # measurement noise covariance matrix
f.Q = process_noise_cov  # process noise covariance matrix

for measurement in measurements:
    f.predict()
    f.update(measurement)
    print(f.x)  # print the state after updating
