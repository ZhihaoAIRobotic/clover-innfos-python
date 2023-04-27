import numpy as np
import mujoco_viewer
from mujoco_py import MjSim, load_model_from_path
import mujoco
import matplotlib.pyplot as plt
import os
from scps.dynamics import gravity, inertia, centrifugalterms
from scps.controller import PD_controller
import pandas as pd


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

xcel_data = pd.read_excel('/home/ubuntu/Github/clover-innfos-python/trajectories/test_traj/test_ilqr3d_03.xlsx')
q_dot = pd.DataFrame(xcel_data, columns=['u1', 'u2', 'u3', 'u4', 'u5', 'u6'])
q_dot = q_dot.to_numpy()

x = pd.DataFrame(xcel_data, columns=['J_dot_1', 'J_dot_2', 'J_dot_3', 'J_dot_4', 'J_dot_5', 'J_dot_6'])
x = x.to_numpy()

# print(x_dot.shape)
traj = x

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0

for i in range(len(traj)):

    # data.ctrl[:] = traj[i]

    # theta_dot = data.qvel[:]

    theta_dot = traj[i]
    theta = data.qpos[:]
    theta_d = np.array([-3.97674561, 50.89657676, 120.01220703, 87.05993652, -7.04406738, 86.52160645])
    theta_d = theta_d * (np.pi / 180)
    #
    u = con.pos_control(theta, theta_d, theta_dot)
    #
    data.qvel[:] = u

    mujoco.mj_step(model, data)
    viewer.render()

    i = i + 1
print('pls')

theta = data.qpos[:]
print(theta)
theta_d = np.array([-3.97674561, 50.89657676, 120.01220703, 87.05993652, -7.04406738, 86.52160645])
theta_d = theta_d*(np.pi/180)
print(theta_d)








