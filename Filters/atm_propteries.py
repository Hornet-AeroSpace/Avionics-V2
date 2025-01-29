import numpy as np

# Air density data taken from http://www.braeunig.us/space/atmos.htm

data = np.genfromtxt("Filters/atmospheric_density.csv", delimiter=",", skip_header=1)

def atmospheric_density(altitude: float):
    return np.interp(altitude, data[:,0], data[:,3])