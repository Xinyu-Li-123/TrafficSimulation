import numpy as np 

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *

def dummy_initialize():
    # compute maximal capacity of each lane based on vehicle parameters


    loc = np.zeros((LANE_NUM, N, 2))    # (lane_index, vehicle_index, x/y)

    # dummy initialization: linearly spaced vehicles
    for i in range(LANE_NUM):
        loc[i, :, 0] = np.linspace(0, D, N)
        loc[i, :, 1] = i

    d = np.zeros((LANE_NUM, N))

    # v = np.abs(np.random.normal(0.0, 1.0, (LANE_NUM, N)) * 25)     # velocity (m/s) (25 m/s = 90 km/h)
    # v = np.random.normal(0.0, 1.0, (LANE_NUM, N)) * 25              # velocity (m/s) (25 m/s = 90 km/h)
    v = np.ones((LANE_NUM, N)) * 25              # velocity (m/s) (25 m/s = 90 km/h)
    a = np.zeros((LANE_NUM, N))
    return loc, d, v, a