import math

class imu:
    GRAVITY = 9.81 #meters/second^2
    def __init__(self, noise_density: float, sample_rate: float):
        """Class for holding information for IMU in the Kalman filter. 
        standard_devation is used for the covarriance matrix in the Kalman Filter

        Args:
            noise_density (float): Noise of the sensor from data sheet. Check units as it can be in m*g/sqrt(Hz) or micro*g/sqrt(Hz)
            sample_rate (float): Sample rate of the sensor in Hz
        """
        self.noise_density = noise_density # be sure to check units when entering 
        self.sample_rate = sample_rate
        self.standard_deviations = self.standard_deviation_calc()
    
    def standard_deviation_calc(self) -> float:
        """Calculates the standard deviatoin of the sensor by the information provided in initiation

        Returns:
            float: Standard deviation of the noise
        """        
        standard_deviation = self.noise_density * math.sqrt(self.sample_rate) * self.GRAVITY 
        return standard_deviation