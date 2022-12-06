"""
Parameters to 
- run the simulation
- draw the animation
"""

import numpy as np

# tunable parameters
N = 50        # number of vehicles
D = 1200           # length of the highway (m)        
detail_range = (D//2 - 200, D//2 + 200) # range of the detailed plot
T = 1200        # simulation time (s)
dt = 0.05       # time step (s)

# animation parameters
display_animation = False
save_animation = False 
speedup = 100
info_step = 5000         # print information every info_step frames
