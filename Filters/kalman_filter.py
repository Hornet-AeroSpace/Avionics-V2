from .state_model import state_model
from .imu import imu

 
def kalman_filter(altitude: float, 
                  velocity_last_estimate: float, 
                  uncertainty_last: float, 
                  variance_last: float,
                  imu: imu, 
                  time_step: float):

    # State Prediction
    velocity_prediction = velocity_last_estimate + (state_model(altitude, velocity_last_estimate) * time_step)
    varriance = variance_last + imu.noise_density**2
    
    # Update
    K = varriance / (varriance * imu.noise_density)
    velocity_estimate = velocity_prediction + (K * (imu.get_sensor_reading - velocity_prediction))
    varriance_new = (1 - K)* varriance
