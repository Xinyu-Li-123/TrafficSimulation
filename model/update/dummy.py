from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
import numpy as np


def dummpy_update(loc, d, v, a, onroad_mask):
    """
    a dummpy update function for testing the first way of updating

    Note that 
    1. this function shouldn't return anything, but update the input variables directly
    2. in-place vs out-of-place operation on numpy array
      All operations on loc, d, v, a should be in-place
      - arr += d_arr is in-place, but arr = arr + d_arr is not
      - arr[:] = arr[:] + num is in-place
    """
    loc[onroad_mask] = loc[onroad_mask] + v[onroad_mask] * dt
    d[1:] = loc[1:] - loc[:-1]
    v[onroad_mask] = v[onroad_mask] + a[onroad_mask]*dt
    a[onroad_mask] = a[onroad_mask]

