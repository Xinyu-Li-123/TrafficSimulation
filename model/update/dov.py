"""
distance-based optimal velocity
"""

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from parameter.model import dov_param
from utils.utils import ONE_TO_ZERO, log_inf

import numpy as np


ov = np.zeros(N)

def _compute_optimal_velocity(d):
    min_mask = d < dov_param.dmin 
    max_mask = d > dov_param.dmax 
    mid_mask = np.logical_not(min_mask & max_mask)
    if np.sum(min_mask) > 0:
        print("d < dmin occurs")
        print(f"d={d}")

    ov[min_mask] = 0
    ov[mid_mask] = vmax * log_inf(d[mid_mask] / dov_param.dmin) / log_inf(dov_param.dmax/dov_param.dmin)
    ov[max_mask] = vmax 


def dov_update(loc, d, v, a):
    _compute_optimal_velocity(d)            # update ov
    loc[:] = (loc + v*dt) % D               # update loc
    d[:] = (loc[ONE_TO_ZERO] - loc) % D     # update d
    # d[0] = (loc[0] - loc[N-1]) % D
    # d[1:] = (loc[1:] - loc[:-1]) % D
    v[:] = ov                               # update v
    # v[:] = (dt * ov + dov.tau*v) / (dt + dov.tau)
