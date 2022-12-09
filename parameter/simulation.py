"""
Parameters to 
- run the simulation
- draw the animation
"""

import numpy as np

# tunable parameters
N = 50        # number of vehicles, N >= 2   
D = 1000           # length of the highway (m)        
detail_range = (D//2 - 200, D//2 + 200) # range of the detailed plot
T = 2400        # simulation time (s)
dt = 0.05       # time step (s)
total_step = int(T/dt) # total number of steps

# update parameters
use_smoothing = True

# animation parameters
animation_types = ['vehicles', 'vt']
animation_type = animation_types[1]
display_animation = True        # show animation on screen
save_animation = False          # save animation to file
draw_animation = display_animation or save_animation    # draw animation

if display_animation and not save_animation:
    if animation_type == 'vehicles':
        print("Displaying animation of vehicles...")
    elif animation_type == 'vt':
        print("Dislaying animation of v-t relation...")
elif save_animation and not display_animation:
    if animation_type == 'vehicles':
        print("Saving animation of vehicles...")
    elif animation_type == 'vt':
        print("Saving animation of v-t relation...")
elif not (display_animation or save_animation):
    print("Not displaying or saving animation...")
else:
    raise ValueError("Invalid animation parameters!")    


speedup = 100
info_step = 5000         # print information every info_step frames

# x-t tracking parameters
xt_track_iteration_step = 10
xt_track_vehicle_range = np.arange(0, max(N, 50), 10)

# v-t tracking parameters
vt_track_iteration_step = 100
vt_track_max_vehicle = 100
if N < vt_track_max_vehicle:
    vt_track_vehicle_range = np.arange(0, N, 1)
else:
    vt_track_vehicle_range = np.linspace(0, N, vt_track_max_vehicle, dtype=int)
