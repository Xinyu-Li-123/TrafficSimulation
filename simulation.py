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

from utils.utils import check_collision, save_param


def init():

    if draw_animation:
        if animation_type == 'vehicles':
            return all_vehicle_locs, detailed_vehicle_locs
        elif animation_type == 'vt':
            return all_vehicle_velocities,

#TODO: speed up animation by including multiple simulation iterations in one animation update
def update(i, draw_animation=True, detect_snake=False):
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
        if draw_animation:
            print(f"min(v)={v.min()*MPS_TO_KMPH:.2f}km/h, max(v)={v.max()*MPS_TO_KMPH:.2f}km/h, mean(v)={v.mean()*MPS_TO_KMPH:.2f}km/h")
    elif i == 0:
        # print(f"d={d}") 
        # print(f"loc={loc}")
        # print(f"v={v*MPS_TO_KMPH}")
        print(f"{i}/{total_step}")
        print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
        if draw_animation:
            print(f"min(v)={v.min()*MPS_TO_KMPH:.2f}km/h, max(v)={v.max()*MPS_TO_KMPH:.2f}km/h, mean(v)={v.mean()*MPS_TO_KMPH:.2f}km/h")

    if i == save_param_at_index:
        save_param(loc, d, v, a, save_param_filename)
        print(f"At {i}/{total_step}, Saved parameters to {save_param_filename}")    # dummy.dummpy_update(loc, d, v, a) 

    dov.dov_update(loc, d, v, a, i, dov_update_type=dov_param.dov_update_type)

    if draw_animation:
        if animation_type == 'vehicles':            
            all_vehicle_locs.set_xdata(loc)
            detailed_vehicle_locs.set_xdata(loc)
        elif animation_type == 'vt':
            all_vehicle_velocities.set_ydata(v*MPS_TO_KMPH)

    # traffic snake detection:
    if detect_snake:
        # find out all v that is less than SMALLNUM
        pass 

    # collision detection  
    if check_collision(d):
        is_collided = True
        collision_step = i
        print(f"Collision occurs at time {i*dt:.2f}/{T:.2f}s, {i+1}/{total_step} steps")
        collided_idx = np.where(d < car_length)[0][0]
        print(f"collided_idx={collided_idx}")


        if draw_animation:
            raise ValueError("A collision between two vehicles occurs.")
        else:
            return

    if np.sum(v < 0) > 0:
        # find negative velocities and their indices
        neg_v = v < 0
        neg_v_idx = np.where(neg_v)[0]
        print(f"neg_v_idx={neg_v_idx}")
        print(f"neg_v={v[neg_v_idx]}")
        raise ValueError("Negative velocity occurs.")

    if draw_animation:
        if animation_type == 'vehicles':
            # Due to blit=True, set_title will only work when the animation is saved as video
            ax[0].set_title(f"0-{D}m (Full view) at time {i*dt:.2f}/{T:.2f}s")
            ax[1].set_title(f"{detail_range[0]}-{detail_range[1]}m at time {i*dt:.2f}/{T:.2f}s")
        elif animation_type == 'vt':
            ax.set_title(f"Time: {i*dt:.2f}/{T:.2f}s")

    if draw_animation:
        if animation_type == 'vehicles':
            return all_vehicle_locs, detailed_vehicle_locs
        elif animation_type == 'vt':
            return all_vehicle_velocities,

# Print Settings
print("DOV setting:")
print(f"\tdmin={dov_param.dmin}m, dmax={dov_param.dmax}m, tau={dov_param.tau}")
print(f"\t{dov_param.dov_update_type} dov update type...\n")
print(f"Running simulation with {N} vehicles, duration {T}s, {dt}s time step\n")

# loc, d, v, a = dummy_initialize()
loc, d, v, a = partial_highway_initialize()
# loc, d, v, a = equidistant_initialize()
# loc, d, v, a = record_initialize("param_50000.pkl")
is_collided = False
collision_step = -1
collided_idx = -1



# animate vehicle movement

if draw_animation:
    for i in range(animation_start_index):
        if is_collided:
            break
        if i % info_step == 0:
            print(f"Time: {(i+1)*dt:.2f}/{T:.2f}s")
            print(f"min(v)={v.min()*MPS_TO_KMPH:.2f}km/h, max(v)={v.max()*MPS_TO_KMPH:.2f}km/h, mean(v)={v.mean()*MPS_TO_KMPH:.2f}km/h")
        update(i, draw_animation=False)

    if animation_type == 'vehicles':

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
        all_vehicle_locs, = ax[0].plot(
            loc, 
            np.ones(N) * LANE_WIDTH / 2, 
            marker="o", markersize=1, ls="None",)
        # for i in range(N):
        #     all_vehicle_locs.set_color(color[i])
        detailed_vehicle_locs, = ax[1].plot(
            loc, 
            np.ones(N) * LANE_WIDTH/2,
            marker="o", markersize= np.min([5, 5 / ((detail_range[1]-detail_range[0])/200)]), ls="None")

        # Print parameters
        print("Simulation time: {}s\ndt: {:.2f}s\n#Update: {}\nAnimation speedup: {}".format(
            T, dt, total_step, speedup))
        print(f"Number of vehicles: {N}")
        print(f"Road length: {D}m")
        print(f"Save animation: {save_animation}")

        ani = animation.FuncAnimation(
            fig, update, init_func=init, 
            frames=range(animation_start_index, total_step), 
            interval=int(dt*1000/speedup), 
            blit=True, repeat=False)

        if save_animation:
            ani.save('./results/animation.mp4', writer='ffmpeg', fps=240)
        else:
            plt.show()
    
    elif animation_type == 'vt':
        fig, ax = plt.subplots(1, 1)      # plot v-t relation and traffic snake

        all_vehicle_velocities, = ax.plot(
            np.arange(N), v*MPS_TO_KMPH,
            marker="o", markersize=5, markerfacecolor='r')

        ax.set_xlim(-1, N*1.1)
        ax.set_ylim(-1, vmax*1.1*MPS_TO_KMPH)
        ax.set_xlabel('Car Index')
        ax.set_ylabel('Velocity (km/h)')
        ax.set_title(f'Velocity distribution at time 0.00/{T:.2f}')
        ax.set_aspect((N*1.1)/((vmax*MPS_TO_KMPH*1.1)*1.2))
        # ax.legend()

        # a red dotted line that indicates maximal velocity
        ax.plot([0, N], [vmax*MPS_TO_KMPH, vmax*MPS_TO_KMPH], 'r--')
        ax.text(N*0.8, vmax*MPS_TO_KMPH*1.01, f"vmax={vmax*MPS_TO_KMPH:.2f}km/h", fontsize=10)

        ani = animation.FuncAnimation(
            fig, update, init_func=init, 
            frames=range(animation_start_index, total_step), 
            interval=int(dt*1000/speedup), 
            blit=True, repeat=False)

        if save_animation:
            ani.save(f'./results/{animation_name}.mp4', writer='ffmpeg', fps=240)
        else:
            plt.show()

elif animation_demo_type == 'summary':
    xt_track_vehicle_loc = np.zeros((total_step//xt_track_iteration_step, N))
    vt_track_vehicle_velocity = np.zeros((total_step//xt_track_iteration_step, N))
    metric_mean_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_min_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_max_velocity = np.zeros(total_step//xt_track_iteration_step)
    metric_std_velocity = np.zeros(total_step//xt_track_iteration_step)
    if dov_param.cmp_to_ov:
        metric_ov = np.zeros((total_step//xt_track_iteration_step, N))

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
            if dov_param.cmp_to_ov:
                metric_ov[i//xt_track_iteration_step, :] = np.copy(dov.ov)

    # save vt_track_vehicle_velocity
    pickle.dump(vt_track_vehicle_velocity, open(f'./record/{dt}_v.pkl', 'wb'))

    # # all_vehicle_locs.set_xdata(loc)
    # # detailed_vehicle_locs.set_xdata(loc)
    # fig2, ax2 = plt.subplots(1, 1)      # plot x-t relation
    # fig3, ax3 = plt.subplots(2, 1)      # plot mean and std of velocity
    # fig4, ax4 = plt.subplots(1, 1)      # plot v-t relation
    # if dov_param.cmp_to_ov:
    #     fig5, ax5 = plt.subplots(1, 2)      # plot difference b/t ov and v

    # # plot x-t relation
    # for n in xt_track_vehicle_range:
    #     xt_track_single_vehicle_loc = np.copy(xt_track_vehicle_loc[:, n])
    #     offset = np.zeros_like(xt_track_single_vehicle_loc)
    #     for j in range(1, xt_track_single_vehicle_loc.shape[0]):
    #         if xt_track_single_vehicle_loc[j] < xt_track_single_vehicle_loc[j-1]:
    #             offset[j:] += D
    #     xt_track_single_vehicle_loc += offset

    #     ax2.plot(
    #         np.linspace(0, T, total_step//xt_track_iteration_step), xt_track_single_vehicle_loc, label=f"Vehicle {n}")

    # ax2.set_xlabel('Time (s)')
    # ax2.set_ylabel('Location (m)')
    # ax2.set_title('Location of vehicles')
    # ax2.legend()

    # # plot mean(v)-t relation
    # ax3[0].plot(
    #     np.linspace(0, T, total_step//xt_track_iteration_step), metric_mean_velocity*MPS_TO_KMPH, label='Mean velocity')
    # # plot min(v)-t relation
    # ax3[0].plot(
    #     np.linspace(0, T, total_step//xt_track_iteration_step), metric_min_velocity*MPS_TO_KMPH, label='Min velocity')
    # # plot max(v)-t relation
    # ax3[0].plot(
    #     np.linspace(0, T, total_step//xt_track_iteration_step), metric_max_velocity*MPS_TO_KMPH, label='Max velocity')
    # # vmax as reference
    # ax3[0].plot(np.linspace(0, T, total_step//xt_track_iteration_step), np.ones(total_step//xt_track_iteration_step)*vmax*MPS_TO_KMPH, 'r--')   
    # ax3[0].set_xlabel('Time (s)')
    # ax3[0].set_ylabel('Velocity (km/h)')
    # ax3[0].set_title('Velocity of vehicles')
    # if dov_param.cmp_to_ov:
    #     # mean of ov(t) (v(t) w/ tau=0) as reference
    #     ax3[0].plot(np.linspace(0, T, total_step//xt_track_iteration_step), np.mean(metric_ov, axis=1)*MPS_TO_KMPH, 'g--', label='Mean ov(t) (v(t) w/ tau=0)')

    # ax3[0].legend()
   
    # # plot std(v)-t relation
    # ax3[1].plot(
    #     np.linspace(0, T, total_step//xt_track_iteration_step), metric_std_velocity*MPS_TO_KMPH)
    # ax3[1].set_xlabel('Time (s)')
    # ax3[1].set_ylabel('Std velocity (km/h)')
    # ax3[1].set_title('Std velocity of vehicles')
    
    # print(f"Mean velocity: {np.mean(v)*MPS_TO_KMPH:.2f}km/h")


    # plot v-t relation
    # for n in xt_track_vehicle_range[0:1]:
    #     vt_track_single_vehicle_velocity = np.copy(vt_track_vehicle_velocity[:, n])
    #     if dov_param.cmp_to_ov:
    #         ax4.plot(
    #             np.linspace(0, T, total_step//xt_track_iteration_step), metric_ov[:, n]*MPS_TO_KMPH, label=f"ov(t) of vehicle {n}")
    #     ax4.plot(
    #         np.linspace(0, T, total_step//xt_track_iteration_step), vt_track_single_vehicle_velocity*MPS_TO_KMPH, label=f"Vehicle {n}")
    # ax4.set_xlabel('Time (s)')
    # ax4.set_ylabel('Velocity (km/h)')
    # ax4.set_title('Velocity of vehicles')
    # ax4.legend()

    # if dov_param.cmp_to_ov:
    #     # # plot difference b/t ov and v of car #N//4
    #     # ax5[0].plot(
    #     #     np.linspace(0, T, total_step//xt_track_iteration_step), (metric_ov[:, n]-vt_track_vehicle_velocity[:, N//4])*MPS_TO_KMPH, label=f"Vehicle {n}")
    #     # ax5[0].set_xlabel('Time (s)')
    #     # ax5[0].set_ylabel('Difference (km/h)')
    #     # ax5[0].set_title('Difference b/t ov and v')
    #     # ax5[0].legend()

    #     # # plot difference b/t ov and v of car #3*N//4
    #     # ax5[1].plot(
    #     #     np.linspace(0, T, total_step//xt_track_iteration_step), (metric_ov[:, n]-vt_track_vehicle_velocity[:, 3*N//4])*MPS_TO_KMPH, label=f"Vehicle {n}")
    #     # ax5[1].set_xlabel('Time (s)')
    #     # ax5[1].set_ylabel('Difference (km/h)')
    #     # ax5[1].set_title('Difference b/t ov and v')
    #     # ax5[1].legend()
    #     for n in xt_track_vehicle_range:
    #         ax5[0].plot(
    #             np.linspace(0, T, total_step//xt_track_iteration_step), 
    #             (metric_ov[:, n]-vt_track_vehicle_velocity[:, n])*MPS_TO_KMPH, 
    #             label=f"Vehicle {n}")
    #         ax5[0].set_xlabel('Time (s)')
    #         ax5[0].set_ylabel('Difference (km/h)')
    #         ax5[0].set_title('Difference b/t ov and v')
    #         ax5[0].legend()
        
    #     # plot difference b/t ov and v of a particular car
    #     ax5[1].plot(
    #         np.linspace(0, T, total_step//xt_track_iteration_step), 
    #         (metric_ov[:, xt_track_vehicle_range[0]]-vt_track_vehicle_velocity[:, xt_track_vehicle_range[0]])*MPS_TO_KMPH, 
    #         label=f"Vehicle {xt_track_vehicle_range[0]}")
    #     ax5[1].set_xlabel('Time (s)')
    #     ax5[1].set_ylabel('Difference (km/h)')
    #     ax5[1].set_title(f'Difference b/t ov and v of car {xt_track_vehicle_range[0]}')
    #     ax5[1].legend()

    plt.show()

elif animation_demo_type == 'diy':

    # is result independent of dt?

    dts = np.array([0.01, 0.025, 0.05])

    for _ in range(len(dts)):
        total_step = int(T/dts[_])
        for i in range(total_step):
            update(i, draw_animation=False)
