from parameter.vehicle import car_length

# dov model
dmin = 0.2 + car_length*3     # if d < dmin, negative velocity is allowed (may not be an error?)
dmax = 100 + car_length*3      # if d > dmax, v = vmax
tau = 0.7