"""
This file mainly deal with the animation. The rest of the utility is
distributed accross different sub-modules
"""

import numpy as np
np.random.seed(11)
import pickle

from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *

from model.update import dummy 
from model.update import dov
from model.initialize import *

from utils.utils import check_collision, save_param, fit_piecewise_linear

#TODO: speed up animation by including multiple simulation iterations in one animation update
def update(i, draw_animation=True):
    global loc, d, v, a, is_collided, collision_step, collided_idx # , onroad_mask
    # fig.canvas.resize_event()
    if (i+1) % info_step == 0 or i == total_step:
        # print(f"d={d}") 
        # print(f"loc={loc}")
        # print(f"v={v*MPS_TO_KMPH}")
        # print(f"d_min={d.min()}")
        # print(f"computed d_min={np.min((loc[ONE_TO_ZERO] - loc))%D}")
        # print(f"mean(v)={v.mean()*MPS_TO_KMPH}, max(v)={v.max()*MPS_TO_KMPH}, min(v)={v.min()*MPS_TO_KMPH}")
        print(f"{i+1}/{total_step}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
        if not draw_animation:
            print(f"min(v)={v.min()*MPS_TO_KMPH:.2f}km/h, max(v)={v.max()*MPS_TO_KMPH:.2f}km/h, mean(v)={v.mean()*MPS_TO_KMPH:.2f}km/h, std(v)={v.std()*MPS_TO_KMPH:.2f}km/h")
            print(f"min(d)={d.min():.2f}m, max(d)={d.max():.2f}m, mean(d)={d.mean():.2f}m, std(d)={d.std():.2f}m")
    elif i == 0:
        # print(f"d={d}") 
        # print(f"loc={loc}")
        # print(f"v={v*MPS_TO_KMPH}")
        print(f"{i}/{total_step}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
        if not draw_animation:
            print(f"min(v)={v.min()*MPS_TO_KMPH:.2f}km/h, max(v)={v.max()*MPS_TO_KMPH:.2f}km/h, mean(v)={v.mean()*MPS_TO_KMPH:.2f}km/h, std(v)={v.std()*MPS_TO_KMPH:.2f}km/h")
            print(f"min(d)={d.min():.2f}m, max(d)={d.max():.2f}m, mean(d)={d.mean():.2f}m, std(d)={d.std():.2f}m")

    if i == save_param_at_index:
        save_param(loc, d, v, a, save_param_filename)
        print(f"At {i}/{total_step}, Saved parameters to {save_param_filename}")    # dummy.dummpy_update(loc, d, v, a) 

    dov.dov_update(loc, d, v, a, i, dov_update_type=dov_param.dov_update_type)

   # traffic snake detection:
   
        # find out all v that is less than SMALLNUM
        # small_v = v < snake_max_distance    
        # all_vehicle_locs[small_v].set_color('r')
        # all_vehicle_locs[np.logical_not(small_v)].set_color('b')
        # detailed_vehicle_locs[small_v].set_color('r')
        # detailed_vehicle_locs[np.logical_not(small_v)].set_color('b')

    # collision detection  
    if check_collision(d):
        is_collided = True
        collision_step = i
        print(f"Collision occurs at time {i*dt:.2f}/{T:.2f}s, {i+1}/{total_step} steps")
        collided_idx = np.where(d < car_length)[0][0]
        print(f"At {i*dt}/{T}, collided_idx={collided_idx}")
        return

    if np.sum(v < 0) > 0:
        # find negative velocities and their indices
        neg_v = v < 0
        neg_v_idx = np.where(neg_v)[0]
        print(f"neg_v_idx={neg_v_idx}")
        print(f"neg_v={v[neg_v_idx]}")
        raise ValueError("Negative velocity occurs.")



def plot_at(ji, ti):
    global loc, d, v, a, ax3, is_collided, collision_step, collided_idx # , onroad_mask
    # Print Settings
    print("DOV setting:")
    print(f"\tdmin={dov_param.dmin}m, dmax={dov_param.dmax}m, tau={dov_param.tau}")
    print(f"\t{dov_param.dov_update_type} dov update type...\n")
    print(f"Running simulation with {N} vehicles, duration {T}s, {dt}s time step\n")

    # loc, d, v, a = dummy_initialize()
    loc, d, v, a = partial_highway_initialize()
    # loc, d, v, a = equidistant_initialize(jitter=init_jitter)
    # loc, d, v, a = record_initialize("param_50000.pkl")

    is_collided = False
    collision_step = -1
    collided_idx = -1

    # animate vehicle movement

    xt_track_vehicle_loc = np.zeros((total_step//xt_track_iteration_step, N))
    vt_track_vehicle_velocity = np.zeros((total_step//xt_track_iteration_step, N))
    metric_mean_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_min_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_max_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_std_velocity = np.zeros(total_step//xt_track_iteration_step)

    for i in range(total_step):
        if is_collided:
            break
        update(i, draw_animation)
        if i % xt_track_iteration_step == 0:
            xt_track_vehicle_loc[i//xt_track_iteration_step, :] = loc
            vt_track_vehicle_velocity[i//xt_track_iteration_step, :] = v
            metric_mean_velocity[i//xt_track_iteration_step] = np.mean(v)
            metric_min_velocity[i//xt_track_iteration_step] = np.min(v)
            metric_max_velocity[i//xt_track_iteration_step] = np.max(v)
            metric_std_velocity[i//xt_track_iteration_step] = np.std(v)

    print(f"Mean velocity: {np.mean(v)*MPS_TO_KMPH:.2f}km/h")
    print("Mean distance: {:.2f}m".format(
        np.mean(d)
    ))

    # all_vehicle_locs.set_xdata(loc)
    # detailed_vehicle_locs.set_xdata(loc)

    # plot mean(v)-t relation
    ax3[ji,ti].plot(
        np.linspace(0, T, total_step//xt_track_iteration_step), metric_mean_velocity*MPS_TO_KMPH, label='Mean velocity')
    # plot min(v)-t relation
    ax3[ji,ti].plot(
        np.linspace(0, T, total_step//xt_track_iteration_step), metric_min_velocity*MPS_TO_KMPH, label='Min velocity')
    # plot max(v)-t relation
    ax3[ji,ti].plot(
        np.linspace(0, T, total_step//xt_track_iteration_step), metric_max_velocity*MPS_TO_KMPH, label='Max velocity')
    # vmax as reference
    ax3[ji,ti].plot(np.linspace(0, T, total_step//xt_track_iteration_step), np.ones(total_step//xt_track_iteration_step)*vmax*MPS_TO_KMPH, 'r--')   
    # ax3[ji,ti].set_xlabel('Time (s)')
    # ax3[ji,ti].set_ylabel('Velocity (km/h)')
    ax3[ji,ti].set_title('Jitter={:.2f}, tau={:.2f}'.format(init_jitter, dov_param.tau))

    if (ji, ti) == (0, 0):
        ax3[ji,ti].legend()


lov, d, v, a = partial_highway_initialize()
# loc, d, v, a = equidistant_initialize(jitter=init_jitter)

jitters = np.array([0.0, 2, 4])
taus = np.array([0.1, 0.5, 1.0])

is_collided = False

fig3, ax3 = plt.subplots(3, 3, figsize=(12,10))      # plot mean and std of velocity
# set hspace
plt.subplots_adjust(hspace=0.3)

test_count = 3
dmins = np.zeros(test_count)
dmaxs = np.zeros(test_count)


# test 1
# dov_param.dmin =  0.2 + car_length*3 
# dov_param.dmax =  50 + car_length*3
# de = D/N
# dmins[0] = dov_param.dmin
# dmaxs[0] = dov_param.dmax
# vmax = vmax
# dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
# ve = dov.ov[0]
# print(dmins[0], dmaxs[0], ve*MPS_TO_KMPH)

# dmins[1] = dov_param.dmin-1.174952
# dmaxs[1] = dov_param.dmax+20
# vmax = vmax
# dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
# ve = dov.ov[0]
# print(dmins[1], dmaxs[1], ve*MPS_TO_KMPH)

# dmins[2] = dov_param.dmin+0.790767
# dmaxs[2] = dov_param.dmax-10
# vmax = vmax
# dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
# ve = dov.ov[0]
# print(dmins[2], dmaxs[2], ve*MPS_TO_KMPH)

# for i in range(test_count):
#     dov_param.dmin = dmins[i]
#     dov_param.dmax = dmaxs[i]

#     plot_at(i, 0)

# plt.show()
# # fig3.savefig('results/jitter-tau.svg')

dov_param.dmin =  0.2 + car_length*3 
dov_param.dmax = 80 + car_length*3
de = D/N
dmins[0] = dov_param.dmin
dmaxs[0] = dov_param.dmax
vmax = vmax
dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
ve = dov.ov[0]
print(dmins[0], dmaxs[0], ve*MPS_TO_KMPH)

dmins[1] = dov_param.dmin-1.174952
dmaxs[1] = dov_param.dmax+20
dov_param.dmin = dmins[1]
dov_param.dmax = dmaxs[1]
vmax = vmax
dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
ve = dov.ov[0]
print(dmins[1], dmaxs[1], ve*MPS_TO_KMPH)

dmins[2] = dov_param.dmin+0.790767
dmaxs[2] = dov_param.dmax-10
dov_param.dmin = dmins[2]
dov_param.dmax = dmaxs[2]
vmax = vmax
dov._compute_optimal_velocity(np.ones(N)*de, dov_param.dov_update_types[0])
ve = dov.ov[0]
print(dmins[2], dmaxs[2], ve*MPS_TO_KMPH)

for i in range(test_count):
    dov_param.dmin = dmins[i]
    dov_param.dmax = dmaxs[i]

    plot_at(i, 0)
plt.show()
# # fig3.savefig('results/jitter-tau.svg')