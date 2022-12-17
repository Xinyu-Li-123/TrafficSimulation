"""
distance-based optimal velocity
"""

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from parameter.model import dov_param
from utils.utils import ONE_TO_ZERO, log_inf, log_inf_approx

import numpy as np

ov = np.zeros(N)

def _compute_optimal_velocity(d, dov_update_type):
    min_mask = d < dov_param.dmin 
    max_mask = d > dov_param.dmax 
    mid_mask = np.logical_not(np.logical_or(min_mask, max_mask))
    ov[min_mask] = 0
    if dov_update_type == dov_param.dov_update_types[0]:
        ov[mid_mask] = \
            vmax * log_inf(d[mid_mask] / dov_param.dmin) / \
            log_inf(dov_param.dmax/dov_param.dmin)
    elif dov_update_type == dov_param.dov_update_types[1]:
        # second order approximation of log_inf
        # seems to fail
        ov[mid_mask] = \
            vmax * log_inf_approx(
                d[mid_mask]/dov_param.dmin, 
        order=dov_param.dov_update_approx_order) / \
            log_inf(dov_param.dmax / dov_param.dmin)
    elif dov_update_type == dov_param.dov_update_types[2]:
        ov[mid_mask] = \
            vmax * (d[mid_mask] - dov_param.dmin) / \
                (dov_param.dmax - dov_param.dmin)
    else:
        raise ValueError('dov_update_type should be one of the following: {}'.format(
            dov_param.dov_update_types))
    ov[max_mask] = vmax 


def dov_update(loc, d, v, a, i, dov_update_type="smoothing"):
    global count
    _compute_optimal_velocity(d, dov_update_type=dov_param.dov_update_type)            # update ov
    loc[:] = (loc + v*dt) % D               # update loc
    d[:] = (loc[ONE_TO_ZERO] - loc) % D     # update d
    v[:] = (dt * ov + dov_param.tau*v) / (dt + dov_param.tau)
    
