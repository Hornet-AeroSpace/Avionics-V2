import numpy as np

# Air density data taken from http://www.braeunig.us/space/atmos.htm

file = np.genfromtxt("Filters/atmospheric_density.csv", delimiter=",", skip_header=1)
print(file[:, 0])
print(file[:, 3])

print(np.interp([0, 100, 1000, 10000, 11000], file[:,0], file[:,3]))
