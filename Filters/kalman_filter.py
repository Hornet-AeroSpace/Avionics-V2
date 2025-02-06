from .state_model import state_model
from .imu import imu

 
def kalman_filter(altitude: float, 
                  velocity_last_estimate: float, 
                  variance_last: float,
                  imu: imu, 
                  time_step: float) -> tuple[float, float]:
    """ Kalman Filter Function: takes the sensor data and compare to a mathmatical model and adjust the sensed data 
    to remove noise and sensor error. 

    Args:
        altitude (float): (meters) hight of the rocket, used for atmospheric drag
        velocity_last_estimate (float): the last corrected data point provided by the filter
        variance_last (float): last varriace value of filter (how much noise and error is expected)
        imu (imu): the senser class
        time_step (float): the time interval inbetween sampeles

    Returns:
        tuple[float, float]: veloctiy_estimate - the corrected value, varriance_new - the new varriace value (likely not to change)
    """
    # State Prediction
    velocity_prediction = velocity_last_estimate + (state_model(altitude, velocity_last_estimate) * time_step)
    varriance = variance_last + imu.noise_density**2 
    
    # Update
    K = varriance / (varriance * imu.noise_density)
    velocity_estimate = velocity_prediction + (K * (imu.acceleration - velocity_prediction))
    
    varriance_new = (1 - K)* varriance
    print(f"varriance {varriance}")
    print(f"k = {K}")
    print(f"new verriance {varriance_new}")
    return velocity_estimate, varriance_new