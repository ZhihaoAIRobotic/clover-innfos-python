import numpy as np
import mujoco_viewer
import mujoco
import matplotlib.pyplot as plt
import os
from scps.dynamics import gravity, inertia, centrifugalterms
from scps.controller import PD_controller
import pandas as pd
import time
from coffee_project.polynomial_trajectory import PolynomialGenerator

pg = PolynomialGenerator()

ASSETS = {}
root_path = '/home/hengyi/GitHub/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

pg = PolynomialGenerator()

q0 = [0, 0, 0, 0, 0, 0]
dq0 = [0, 0, 0, 0, 0, 0]
ddq0 = [0, 0, 0, 0, 0, 0]
q_start = np.array([q0, dq0, ddq0])
via_points = np.array([[0, np.pi/4, -np.pi/4, 0, 0, 0],
                       [0, np.pi/2, -np.pi/2, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0]])
t_list = np.array([1.0, 1.50, 2.5])
n_list = np.array([500, 250, 500])

q, dq, ddq = pg.generate_trajectory(q_start, via_points, 0, t_list, n_list)

viewer = mujoco_viewer.MujocoViewer(model, data)

Kp = 0.7
Kd = 0.7

i = 0

for i in range(len(q)):
    theta = data.qpos
    theta_dot = data.qvel
    # data.ctrl[:] = inertia(ddq[i])
    u = gravity(theta) + Kp*(q[i] - theta) - Kd*(dq[i] - theta_dot)
    data.qvel = u


    mujoco.mj_step(model, data)
    viewer.render()

    i = i + 1