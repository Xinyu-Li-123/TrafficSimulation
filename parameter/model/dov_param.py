from parameter.vehicle import car_length

# dov model
# If dov_update_type == "smoothing", d_max - d_min should not equal to 0 b/c of the use of logarithm
dmin = 0.2 + car_length*3     # if d < dmin, negative velocity is allowed (may not be an error?)
dmax = 100 + car_length*3      # if d > dmax, v = vmax
tau = 0.5      # times to equilibrium grows exponentially(?) with tau until some critical value

dov_update_types = ["smoothing", "approximation", "linear"]
dov_update_type = dov_update_types[0]
dov_update_approx_order = 3
if dov_update_type == dov_update_types[1]:
    if dov_update_approx_order < 1:
        raise ValueError("dov_update_approx_order must be >= 1!")
    elif dov_update_approx_order % 2 == 0:
        raise ValueError("dov_update_approx_order must be odd!")

cmp_to_ov = True