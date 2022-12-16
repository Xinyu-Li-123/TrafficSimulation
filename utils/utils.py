"""
This file stores parameters for the simulation and some utility functions.

To avoid circular import, this file should not import any other files besides paraemters,
and files in parameter should not import any files in module utils
"""

import numpy as np
import pickle
import os

from parameter.vehicle import *
from parameter.simulation import *

# conversion between km/h and m/s
# There should only be two conversions throughout the simulation:
# - km/h to m/s during initialization (initialize using km/h, simulate using m/s)
# - m/s to km/h during outputing statistics (people are more familiar with km/h)


ONE_TO_ZERO = np.array(list(range(1, N)) + [0])   # to loop back from 1 to N-1 to 0 when calculating distance

SMALLNUM = 1e-2
BIGNUM = 1e6

print(car_length)

def check_collision(d):
    return np.sum(d <= car_length) > 0

def find_traffic_snake(v):
    """
    Find traffic snake by finding the longest sequence of vehicles that are still.
    """
    ans = np.where(v <= SMALLNUM)
    if ans.size != 0:
        print(ans)


def log_inf(x):
    x = np.copy(x)
    min_mask = x < SMALLNUM
    x[min_mask] = -BIGNUM     # when x is too small, use a large negative number instead
    x[np.logical_not(min_mask)] = np.log(x[np.logical_not(min_mask)])
    return x

def log_inf_approx(x, order=3):
    """
    Approximate log_inf by a polynomial of the given order.
    """
    x = np.copy(x)
    min_mask = x < SMALLNUM
    x[min_mask] = -BIGNUM     
    if order % 2 == 0:
        raise ValueError('order should be odd')
    elif order == 1:
        x[np.logical_not(min_mask)] = 1 - x[np.logical_not(min_mask)]
    elif order == 3:
        x[np.logical_not(min_mask)] = \
            -(1-x[np.logical_not(min_mask)]) \
            -(1-x[np.logical_not(min_mask)])**2/2 \
            -(1-x[np.logical_not(min_mask)])**3/6
    else:
        raise ValueError('order should be 1 or 2')
    return x

def save_param(loc, d, v, a, path):
    """
    Save loc, d, v, a to a file.
    """
    if not os.path.exists('record'):
        os.mkdir('record')
    path = os.path.join('record', path)
    with open(path, 'wb') as f:
        pickle.dump((loc, d, v, a), f)
