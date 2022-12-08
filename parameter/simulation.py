"""
Parameters to 
- run the simulation
- draw the animation
"""

import numpy as np

# tunable parameters
N = 50        # number of vehicles      
D = 1000           # length of the highway (m)        
detail_range = (D//2 - 200, D//2 + 200) # range of the detailed plot
T = 2400        # simulation time (s)
dt = 0.05       # time step (s)
total_step = int(T/dt) # total number of steps

# update parameters
use_smoothing = True

# animation parameters
display_animation = False
save_animation = False 
speedup = 100
info_step = 5000         # print information every info_step frames

# tracking parameters
xt_track_iteration_step = 10
# tracking x-t relation
xt_track_vehicle_range = np.arange(0, max(N, 50), 10)

# tracking v-t relation
vt_track_max_vehicle = 100
if N < vt_track_max_vehicle:
    vt_track_vehicle_range = np.arange(0, N, 1)
else:
    vt_track_vehicle_range = np.linspace(0, N, vt_track_max_vehicle, dtype=int)
