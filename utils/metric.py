"""
This file gathers metrics that evaluate the convergence of the simulation. The metrics are computed at each time step and stored in a list. The list is then used to plot the convergence level of the simulation.
"""

from parameter.simulation import *
from parameter.vehicle import *
from parameter.road import *

from numpy import *

def density(loc):
    """
    Compute the density of the road at each time step
    """
    return loc.size / D