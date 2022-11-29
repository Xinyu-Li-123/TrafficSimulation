import numpy as np 

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from utils.utils import KMPH_to_MPS, MPS_TO_KMPH

def dummy_initialize():
    # compute maximal capacity of each lane based on vehicle parameters
    loc = np.zeros(N, dtype=float)
    loc = np.random.random(N)*D - D//2
    d = np.zeros(N, dtype=float)
    d[0] = np.inf           # no car in front of 0th car (first car entering highway)      
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
    loc[:] = (np.arange(N-1, -1, -1)*5 - 5*N + D // 2)    # evenly-spaced with a gap of 5m
    d = np.zeros(N, dtype=float)
    d[0] = np.inf 
    d[1:] = loc[1:] - loc[:-1]
    v = np.zeros(N, dtype=float)
    v[:] = np.ones(N) * 25
    # v[:] = np.ones(N) * 25 + 2*(np.random.random(N)-0.5)*5
    a = np.zeros(N, dtype=float)
    return loc, d, v, a