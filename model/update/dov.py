"""
distance-based optimal velocity
"""

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
import parameter.model.dov as dov
import numpy as np


ov = np.zeros(N)

def _compute_optimal_velocity(d, onroad_mask):
    min_mask = d < dov.dmin & onroad_mask
    max_mask = d > dov.dmax & onroad_mask
    mid_mask = min_mask & max_mask
    ov[min_mask] = 0
    ov[mid_mask] = vmax * np.log(d[mid_mask] / dov.dmin) / np.log(dov.dmax/dov.dmin)
    ov[max_mask] = vmax 


def dov_update(loc, d, v, a, onroad_mask):
    _compute_optimal_velocity(d, onroad_mask)
    loc[onroad_mask] = (loc[onroad_mask] + v[onroad_mask]*dt)
    d[1:] = loc[1:] - loc[:-1]
    v[onroad_mask] = (dt * ov[onroad_mask] + dov.tau*v[onroad_mask]) / (dt + dov.tau)

