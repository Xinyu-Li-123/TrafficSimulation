"""
This file stores parameters for the simulation and some utility functions.
"""

import numpy as np
from parameter.vehicle import *

# conversion between km/h and m/s
# There should only be two conversions throughout the simulation:
# - km/h to m/s during initialization (initialize using km/h, simulate using m/s)
# - m/s to km/h during outputing statistics (people are more familiar with km/h)
KMPH_to_MPS = 1000 / 3600
MPS_TO_KMPH = 1 / KMPH_to_MPS

def is_collided(d):
    pass 