from parameter.vehicle import car_length

# dov model
dmin = 0.2 + car_length     # if d < dmin, negative velocity is allowed (may not be an error?)
dmax = 20 + car_length
tau = 0