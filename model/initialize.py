import numpy as np 

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from utils.utils import KMPH_to_MPS, ONE_TO_ZERO
from parameter.model import dov_param 

def dummy_initialize():
    # compute maximal capacity of each lane based on vehicle parameters
    loc = np.linspace(0, D-car_length*2, N)
    d = np.zeros(N, dtype=float)
    d[:] = (loc[ONE_TO_ZERO] - loc) % D  
    v = np.ones(N, dtype=float) * 25
    a = np.zeros(N, dtype=float)
    # a = np.random.random(N) * 100
    return loc, d, v, a

#TODO: abnormal update when using this initialization
# only update 
def partial_highway_initialize():
    """
    Part of a highway. No need to speed up / down when entering / exiting highway.
    """
    loc = np.zeros(N, dtype=float)
    # loc[:] = np.linspace(0, D-5, N) + np.random.rand(*loc.shape)*2
    loc[:] = np.linspace(0, (car_length+0.5)*N, N)
    d = np.zeros(N, dtype=float) 
    # d[0] = loc[N-1] - loc[0]
    # d[1:] = loc[1:] - loc[:-1]
    d[:] = (loc[ONE_TO_ZERO] - loc) % D
    v = np.zeros(N, dtype=float)
    v[:] = 0
    v *= KMPH_to_MPS
    # v[:] = np.ones(N) * 25 + 2*(np.random.random(N)-0.5)*5
    a = np.zeros(N, dtype=float)
    return loc, d, v, a