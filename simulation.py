"""
This file mainly deal with the animation. The rest of the utility is
distributed accross different sub-modules
"""

from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
import numpy as np

from parameter.road import *
from parameter.vehicle import *
from parameter.simulation import *

from model.update import dummy 
from model.update import dov
from model.initialize import *

from utils.utils import ONE_TO_ZERO, MPS_TO_KMPH, SMALLNUM

np.random.seed(11)


def init():
    return all_vehicles, detailed_vehicles

#TODO: speed up animation by including multiple simulation iterations in one animation update
def update(i, update_axis=True):
    global loc, d, v, a # , onroad_mask
    # fig.canvas.resize_event()
    if (i+1) % info_step == 0:
        # print(f"d={d}") 
        # print(f"loc={loc}")
        # print(f"v={v*MPS_TO_KMPH}")
        # print(f"d_min={d.min()}")
        # print(f"computed d_min={np.min((loc[ONE_TO_ZERO] - loc))%D}")
        print(f"mean(v)={v.mean()*MPS_TO_KMPH}, max(v)={v.max()*MPS_TO_KMPH}, min(v)={v.min()*MPS_TO_KMPH}")
        print(f"{i+1}/{int(T/dt)}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
    elif i == 0:
        # print(f"d={d}") 
        # print(f"loc={loc}")
        # print(f"v={v*MPS_TO_KMPH}")
        print(f"{i}/{int(T/dt)}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")


    # dummy.dummpy_update(loc, d, v, a) 

    dov.dov_update(loc, d, v, a)

    if update_axis:
        # update ax[0] and ax[1]
        all_vehicles.set_xdata(loc)
        detailed_vehicles.set_xdata(loc)

    # collision detection  
    if np.sum(d <= car_length) > 0:
        print("\n\nA collision between two vehicles occurs.\n\n")

    
    if np.sum(v < 0) > 0:
        # find negative velocities and their indices
        neg_v = v < 0
        neg_v_idx = np.where(neg_v)[0]
        print(f"neg_v_idx={neg_v_idx}")
        print(f"neg_v={v[neg_v_idx]}")
        raise ValueError("Negative velocity occurs.")

    if update_axis:
        # Due to blit=True, set_title will only work when the animation is saved as video
        ax[0].set_title(f"0-{D}m (Full view) at time {i*dt:.2f}/{T:.2f}s")
        ax[1].set_title(f"{detail_range[0]}-{detail_range[1]}m at time {i*dt:.2f}/{T:.2f}s")
    return all_vehicles, detailed_vehicles


# loc, d, v, a = dummy_initialize()
loc, d, v, a = partial_highway_initialize(1)
# onroad_mask = np.zeros(N, dtype=bool)
# onroad_mask = np.ones(N, dtype=bool)
# onroad_mask = np.random.random(N) < 0.1

# animate vehicle movement


fig, ax = plt.subplots(2, 1)

ax[0].set_xlim(-D*0.1, D*1.1)
ax[0].set_ylim(-1, LANE_WIDTH+1)
ax[0].set_xlabel('Distance (m)')
ax[0].set_ylabel('Lane')
ax[0].grid(False)
ax[0].set_aspect(60)
ax[0].plot([0, D], [LANE_WIDTH, LANE_WIDTH], 'k')
ax[0].set_title(f'All vehicles at time 0.00/{T:.2f}s')
# plot a rectangle to represent the lane
ax[0].add_patch(
    Rectangle((0, 0), D, LANE_WIDTH, fill=False, edgecolor='k', linewidth=1)
    )
# plot a rectangle at detailed area
ax[0].add_patch(
    Rectangle((detail_range[0], 0), detail_range[1]-detail_range[0], LANE_WIDTH, fill=False, edgecolor='r', linewidth=2)
    )

# display 900m - 1100m of the highway in detail
ax[1].set_xlim(*detail_range)
ax[1].set_ylim(-1, LANE_WIDTH+1)
ax[1].set_xlabel('Distance (m)')
ax[1].set_ylabel('Lane')
ax[1].set_title('Three-lane highway')
ax[1].grid(False)
ax[1].set_aspect(20)
ax[1].add_patch(
    Rectangle((detail_range[0], 0), detail_range[1]-detail_range[0], LANE_WIDTH, fill=False, edgecolor='r', linewidth=2)
    )
# turn off scale on x and y axes
# ax.get_xaxis().set_visible(False)
# ax.get_yaxis().set_visible(False)
ax[1].set_title(f"{detail_range[0]}-{detail_range[1]}m at time 0.00/{T:.2f}s")

# plot location of vehicles
# scatter with alternating colors of red and blue
all_vehicles, = ax[0].plot(
    loc, 
    np.ones(N) * LANE_WIDTH / 2, 
    marker="o", markersize=1, ls="None",)
# for i in range(N):
#     all_vehicles.set_color(color[i])
detailed_vehicles, = ax[1].plot(
    loc, 
    np.ones(N) * LANE_WIDTH/2,
    marker="o", markersize= np.min([5, 5 / ((detail_range[1]-detail_range[0])/200)]), ls="None")

# Print parameters
print("Simulation time: {}s\ndt: {:.2f}s\n#Update: {}\nAnimation speedup: {}".format(
    T, dt, int(T/dt), speedup))
print(f"Number of vehicles: {N}")
print(f"Road length: {D}m")
print(f"Save animation: {save_animation}")

if display_animation or save_animation:
    ani = animation.FuncAnimation(fig, update, init_func=init, frames=int(T/dt), interval=int(dt*1000/speedup), blit=True, repeat=False)
    if save_animation:
        ani.save('./results/animation.mp4', writer='ffmpeg', fps=240)
    else:
        plt.show()
else:
    track_vehicle_loc = np.zeros((int(T/dt)//100, N))
    for i in range(int(T/dt)):
        update(i, update_axis=False)
        if i % 100 == 0:
            track_vehicle_loc[i//100, :] = loc
    
    all_vehicles.set_xdata(loc)
    detailed_vehicles.set_xdata(loc)
    fig2, ax2 = plt.subplots(1, 1)
    for n in range(N-20, N, 2):
        track_single_vehicle_loc = track_vehicle_loc[:, n]

        offset = np.zeros_like(track_single_vehicle_loc)
        for j in range(1, track_single_vehicle_loc.shape[0]):
            if track_single_vehicle_loc[j] < track_single_vehicle_loc[j-1]:
                offset[j:] += D
        track_single_vehicle_loc += offset
        ax2.plot(
            np.linspace(0, T, int(T/dt)//100), track_single_vehicle_loc, label=f"Vehicle {n}")
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Location (m)')
    ax2.set_title('Location of vehicles')
    ax2.legend()

    plt.show()
