"""
Parameters to 
- run the simulation
- draw the animation
"""

import numpy as np

# numeerical parameters
N = 50        # number of vehicles, N >= 2   
D = 1000           # length of the highway (m)        
detail_range = (D//2 - 200, D//2 + 200) # range of the detailed plot
T = 1000        # simulation time (s)
dt = 0.0025       # time step (s)
total_step = int(T/dt) # total number of steps


# animation parameters
animation_types = ['vehicles', 'vt']
animation_type = animation_types[1]

animation_demo_types = ['display', 'save', 'summary', "diy"]
animation_demo_type = animation_demo_types[2]
display_animation = animation_demo_type == 'display'    # show animation on screen
save_animation = animation_demo_type == 'save'      # save animation to file
draw_animation = display_animation or save_animation    # draw animation
animation_name = "ani_vt_smooth_equilibirum"
animation_start_index = 0       # start animation at this step

# save parameters
save_param_at_index = -1    # save parameters at this step
save_param_filename = "param_{}.pkl".format(save_param_at_index)

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
    print("Summarizing the simulation...")
else:
    raise ValueError("Invalid animation parameters!")    


speedup = 100
info_step = 50000         # print information every info_step frames

# x-t tracking parameters
xt_track_iteration_step = 10
xt_track_max_vehicle = 5
if N < xt_track_max_vehicle:
    xt_track_vehicle_range = np.arange(0, N, 1)
else:
    xt_track_vehicle_range = np.linspace(0, N-1, xt_track_max_vehicle, dtype=int)
xt_track_vehicle_range = np.array([5, 6, 7])

# v-t tracking parameters
vt_track_iteration_step = 100
vt_track_max_vehicle = 100
if N < vt_track_max_vehicle:
    vt_track_vehicle_range = np.arange(0, N, 1)
else:
    vt_track_vehicle_range = np.linspace(0, N-1, vt_track_max_vehicle, dtype=int)
