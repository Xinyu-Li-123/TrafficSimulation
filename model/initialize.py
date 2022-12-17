import numpy as np 
import pickle
import os

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *
from utils.utils import ONE_TO_ZERO, SMALLNUM
from parameter.model import dov_param 

def dummy_initialize():
    # compute maximal capacity of each lane based on vehicle parameters
    loc = np.linspace(0, N*(car_length+SMALLNUM), N)
    d = np.zeros(N, dtype=float)
    d[:] = (loc[ONE_TO_ZERO] - loc) % D  
    v = np.ones(N, dtype=float) * 25
    a = np.zeros(N, dtype=float)
    # a = np.random.random(N) * 100
    return loc, d, v, a

def partial_highway_initialize():
    """
    Part of a highway. No need to speed up / down when entering / exiting highway.
    Evenly spaced with a random jitter in the range of [-jitter, jitter]
    """
    # Check if there are too much cars
    MAX_CAR_NUM = int(D / car_length)
    if N > MAX_CAR_NUM:
        raise ValueError(
            f"Too many vehicles for the given road length\n"
            f"For D={D}m, the maximal number of vehicles is {MAX_CAR_NUM}\n")

    loc = np.zeros(N, dtype=float)
    # loc[:] = np.linspace(0, D-5, N) + np.random.rand(*loc.shape)*2
    # loc[:] = np.linspace(0, (car_length+2*jitter)*N, N) + 2*(np.random.rand(*loc.shape)-0.5)*jitter
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

def equidistant_initialize(jitter=0):
    """
    Vehicles are equally spaced on the circle with a random jitter in the range of [-jitter, jitter]
    """
    # Check if there are too much cars
    MAX_CAR_NUM = int(D / car_length)
    if N > MAX_CAR_NUM:
        raise ValueError(
            f"Too many vehicles for the given road length\n"
            f"For D={D}m, the maximal number of vehicles is {MAX_CAR_NUM}\n")

    loc = np.zeros(N, dtype=float)
    # loc[:] = np.linspace(0, D-5, N) + np.random.rand(*loc.shape)*2
    # loc[:] = np.linspace(0, (car_length+2*jitter)*N, N) + 2*(np.random.rand(*loc.shape)-0.5)*jitter
    loc[:] = np.linspace(0, D, N+1)[:-1]
    loc[:] += 2*(np.random.rand(*loc.shape)-0.5)*jitter
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

def record_initialize(path):
    """
    Use recorded data to initialize the simulation
    """
    path = os.path.join('record', path)
    loc, d, v, a = pickle.load(open(path, 'rb'))
    return loc, d, v, a