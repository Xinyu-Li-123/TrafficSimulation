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

def _compute_optimal_velocity(d, use_smoothing):
    min_mask = d < dov_param.dmin 
    max_mask = d > dov_param.dmax 
    mid_mask = np.logical_not(np.logical_or(min_mask, max_mask))
    ov[min_mask] = 0
    if use_smoothing:
        ov[mid_mask] = vmax * log_inf(d[mid_mask] / dov_param.dmin) / log_inf(dov_param.dmax/dov_param.dmin)
    else:
        ov[mid_mask] = vmax * (d[mid_mask] - dov_param.dmin) / (dov_param.dmax - dov_param.dmin)
    ov[max_mask] = vmax 


def dov_update(loc, d, v, a, use_smoothing=True):
    global count
    _compute_optimal_velocity(d, use_smoothing=use_smoothing)            # update ov
    loc[:] = (loc + v*dt) % D               # update loc
    d[:] = (loc[ONE_TO_ZERO] - loc) % D     # update d
    # v[:] = ov                               # update v
    v[:] = (dt * ov + dov_param.tau*v) / (dt + dov_param.tau)

def dov_smart_update(loc, d, v, a, use_smoothing=True):
    _compute_optimal_velocity(d, use_smoothing=use_smoothing)            # update ov
    loc[:] = (loc + v*dt) % D               # update loc
    d[:] = (loc[ONE_TO_ZERO] - loc) % D     # update d
    # v[:] = ov                               # update v
    # TODO: Use front and back 5 distances to determine the current speed
