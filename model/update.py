from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *

import numpy as np

"""
There are two ways to update the location of vehicles:
1. update only based on current time step
    The order of update is
        update location based on current distance, velocity and acceleration
        update distance based on current location
        update velocity based on current distance, velocity and acceleration
        update acceleration based on current distance, velocity and acceleration
2. update based on current and previous time step
"""

def dummpy_update(loc, d, v, a):
    """
    a dummpy update function for testing the first way of updating

    Note that 
    1. this function shouldn't return anything, but update the input variables directly
    2. in-place vs out-of-place operation on numpy array
      All operations on loc, d, v, a should be in-place
      - arr += d_arr is in-place, but arr = arr + d_arr is not
      - arr[:] = arr[:] + num is in-place
    """
    loc[:, :, 0] = (loc[:,:,0] + v * dt) % D
    
    v += a * dt


