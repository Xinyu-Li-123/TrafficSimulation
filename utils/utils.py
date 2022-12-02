"""
This file stores parameters for the simulation and some utility functions.
"""

import numpy as np
from parameter.vehicle import *
from parameter.simulation import *

# conversion between km/h and m/s
# There should only be two conversions throughout the simulation:
# - km/h to m/s during initialization (initialize using km/h, simulate using m/s)
# - m/s to km/h during outputing statistics (people are more familiar with km/h)
KMPH_to_MPS = 1000 / 3600
MPS_TO_KMPH = 1 / KMPH_to_MPS

ONE_TO_ZERO = np.array(list(range(1, N)) + [0])   # to loop back from 1 to N-1 to 0 when calculating distance

def is_collided(d):
    pass 

#TODO: Log may yield NaN
# This function is used so that log(~0) = -inf instead of NaN
# Source: https://stackoverflow.com/questions/18191273/numpy-log-with-inf-not-nans

SMALLNUM = 1e-2
BIGNUM = 1e6

def log_inf(x):
    x = np.copy(x)
    min_mask = x < SMALLNUM
    x[min_mask] = -BIGNUM     # when x is 
    x[np.logical_not(min_mask)] = np.log(x[np.logical_not(min_mask)])
    return x