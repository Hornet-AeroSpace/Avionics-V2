import math

class imu:
    GRAVITY = 9.81 #meters/second^2
    def __init__(self, noise_density, sample_rate):
        self.noise_density = noise_density # be sure to check units when entering 
        self.sample_rate = sample_rate
        self.standard_deviations = self.standard_deviation_calc()
    
    def standard_deviation_calc(self):
        standard_deviation = self.noise_density * math.sqrt(self.sample_rate) * self.GRAVITY 
        return standard_deviation
