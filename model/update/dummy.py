from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from utils.utils import ONE_TO_ZERO
import numpy as np


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
    loc[:] = (loc + v * dt) % D 
    d[:] = (loc[ONE_TO_ZERO] - loc) % D
    # d[0] = (loc[N-1] - loc[0]) % D
    # d[1:] = (loc[:-1] - loc[1:]) % D
    v[:] = v + a*dt
    a[:] = a

