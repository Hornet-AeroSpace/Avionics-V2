import math

class imu:
    GRAVITY = 9.81 #meters/second^2
    def __init__(self, noise_density, sample_rate):
        self.noise_density = noise_density # be sure to check units when entering 
        self.sample_rate = sample_rate
        self.sigma = self.sigma_calc()
    
    def sigma_calc(self):
        sigma = self.noise_density * math.sqrt(self.sample_rate) * self.GRAVITY 
        return sigma
