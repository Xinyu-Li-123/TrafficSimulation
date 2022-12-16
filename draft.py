from utils.utils import save_param
from model.initialize import record_initialize
import numpy as np
import pickle 
from parameter.simulation import T, xt_track_iteration_step
import matplotlib.pyplot as plt

dts = np.array([0.0025, 0.005, 0.01, 0.025, 0.05])
min_dt = np.min(dts)
max_dt = np.max(dts)

all_v = np.zeros(
    (len(dts), int(T/max_dt/xt_track_iteration_step)), 
    dtype=np.float64)
print(all_v.shape)
for _ in range(len(dts)):
    dt = dts[_]
    step = int(max_dt/dt)
    with open('record/{}_v.pkl'.format(dt), 'rb') as f:
        v = pickle.load(f)
    # print(v.shape, step, xt_track_iteration_step)
    all_v[_,:] = np.copy(v[::step, _])

# compute difference to dt=0.0025
for _ in range(len(dts)):
    print("dt={}, err={:.4f}".format(
        dts[_], 
        np.linalg.norm(all_v[_,:] - all_v[0,:])/all_v.shape[1]))


# plot all_v and differnence to dt=0.0025
fig, ax = plt.subplots(1, 2, figsize=(8, 6))
for _ in range(len(dts)):
    ax[0].plot(all_v[_,:], label='dt={}'.format(dts[_]))
    ax[1].plot(all_v[_,:] - all_v[0,:], label='dt={}'.format(dts[_]))
ax[0].legend()
ax[1].legend()
plt.show()

print("Done!")

