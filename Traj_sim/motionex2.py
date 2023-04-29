import numpy as np
import mujoco_viewer
from mujoco_py import MjSim, load_model_from_path
import mujoco
import matplotlib.pyplot as plt
import os
from scps.dynamics import gravity, inertia, centrifugalterms
from scps.controller import PD_controller
import pandas as pd
import time


ASSETS = {}
root_path = '/home/ubuntu/Github/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/ubuntu/Github/clover-innfos-python/environment models/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

con = PD_controller()
con.kd = 1
con.kp = 1

xcel_data = pd.read_excel('/home/ubuntu/Github/clover-innfos-python/trajectories/test_traj/test_ilqr3d_002s.xlsx')
u = pd.DataFrame(xcel_data, columns=['u1', 'u2', 'u3', 'u4', 'u5', 'u6'])
u = u.to_numpy()

x = pd.DataFrame(xcel_data, columns=['J_dot_1', 'J_dot_2', 'J_dot_3', 'J_dot_4', 'J_dot_5', 'J_dot_6'])
x = x.to_numpy()

# print(x_dot.shape)
traj = u
Kt = 100

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0

for i in range(len(traj)):

    q_vel = data.qvel[:]
    q_pos = data.qpos[:]

    traj = np.linspace(q_vel, u[i], 10)
    path = np.linspace(q_pos, x[i], 10)

    for j in range(10):

        data.qvel[:] = traj[j]

        q = data.qpos[:]
        q_dot = data.qvel[:]

        q_vel = q_dot + Kt * (path[j] - q)

        data.qvel[:] = q_vel

        mujoco.mj_step(model, data)
        viewer.render()

    i = i + 1
print('pls')

theta = data.qpos[:]
print(theta)
theta_d = x[-2]
print(theta_d)








