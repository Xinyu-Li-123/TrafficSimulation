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

from model.update import *
from model.initialize import *

np.random.seed(11)

loc, d, v, a = dummy_initialize()

loc_agg = loc.view().reshape(-1, 2)     # aggregate all lanes

# animate vehicle movement
fig, ax = plt.subplots(2, 1)

ax[0].set_xlim(0, D)
ax[0].set_ylim(-1, LANE_WIDTH * LANE_NUM+1)
ax[0].set_xlabel('Distance (m)')
ax[0].set_ylabel('Lane')
ax[0].set_title('Three-lane highway')
ax[0].grid(False)
ax[0].set_aspect(20)
for i in range(LANE_NUM+1):
    ax[0].plot([0, D], [i * LANE_WIDTH, i * LANE_WIDTH], 'k')
ax[0].set_title(f'All vehicles at time 0.00/{T:.2f}s')
# plot a rectangle at detailed area
ax[0].add_patch(
    Rectangle((detail_range[0], -1), detail_range[1]-detail_range[0], LANE_WIDTH*LANE_NUM+2, fill=False, edgecolor='r', linewidth=2))

# display 900m - 1100m of the highway in detail
ax[1].set_xlim(*detail_range)
ax[1].set_ylim(-1, LANE_WIDTH * LANE_NUM+1)
ax[1].set_xlabel('Distance (m)')
ax[1].set_ylabel('Lane')
ax[1].set_title('Three-lane highway')
ax[1].grid(False)
ax[1].set_aspect(0.5)
for i in range(LANE_NUM+1):
    ax[1].plot([0, D], [i * LANE_WIDTH, i * LANE_WIDTH], 'k')
# turn off scale on x and y axes
# ax.get_xaxis().set_visible(False)
# ax.get_yaxis().set_visible(False)
ax[1].set_title(f"{detail_range[0]}-{detail_range[1]}m at time 0.00/{T:.2f}s")

# plot location of vehicles
all_vehicles, = ax[0].plot(
    loc_agg[:, 0], 
    loc_agg[:, 1]*LANE_WIDTH + LANE_WIDTH/2, 
    marker="o", markersize=1, ls="None") 
detailed_vehicles, = ax[1].plot(
    loc_agg[:, 0], 
    loc_agg[:, 1]*LANE_WIDTH + LANE_WIDTH/2,
    marker="o", markersize= np.min([5, 5 / ((detail_range[1]-detail_range[0])/200)]), ls="None")

def init():
    return all_vehicles, detailed_vehicles

def update(i):
    global loc, d, v, a
    # fig.canvas.resize_event()
    if (i+1) % 100 == 0:
        print(f"{i+1}/{int(T/dt)}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
    elif i == 0:
        print(f"{i}/{int(T/dt)}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")

    dummpy_update(loc, d, v, a)

    # update ax[0] and ax[1]
    all_vehicles.set_data(loc_agg[:, 0], loc_agg[:, 1]*LANE_WIDTH + LANE_WIDTH/2)
    detailed_vehicles.set_data(
        loc_agg[:, 0],
        loc_agg[:, 1]*LANE_WIDTH + LANE_WIDTH/2)

    # collision detection



    # Due to blit=True, set_title will only work when the animation is saved as video
    ax[0].set_title(f"0-{D}m (Full view) at time {i*dt:.2f}/{T:.2f}s")
    ax[1].set_title(f"{detail_range[0]}-{detail_range[1]}m at time {i*dt:.2f}/{T:.2f}s")
    return all_vehicles, detailed_vehicles



print("Simulation time: {}s\ndt: {:.2f}s\n#Update: {}\nAnimation speedup: {}".format(
    T, dt, int(T/dt), speedup))

ani = animation.FuncAnimation(fig, update, init_func=init, frames=int(T/dt), interval=int(dt*1000/speedup), blit=True, repeat=False)
if save_animation:
    ani.save('./results/animation.mp4', writer='ffmpeg', fps=30)
else:
    plt.show()

