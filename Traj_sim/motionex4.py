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
from coffee_project.polynomial_trajectory_cartesian import PolynomialGeneratorCart

pg = PolynomialGeneratorCart()

ASSETS = {}
root_path = '/home/ubuntu/Github/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/ubuntu/Github/clover-innfos-python/environment models/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

x0 = [0, 0, 0, 0, 0, 0]
dx0 = [0, 0, 0, 0, 0, 0]
ddx0 = [0, 0, 0, 0, 0, 0]
x_start = np.array([x0, dx0, ddx0])

via_points = np.array([[0.07568567, -0.06065414, 0.5092126, -0.10471976, 0, 0],
                       [0.07569469, -0.17449179, 0.359509, 0.5, 0, 0],
                       [0.07568567, -0.1744927, 0.359509, 0, 0, 0]])

t_list = np.array([10.0, 15.0, 20.0])
n_list = np.array([100, 50, 50])

q, dq, ddq = pg.generate_trajectory(x_start, via_points, 0, t_list, n_list)

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0

for i in range(len(q)):

    data.qvel[:] = q[i]

    mujoco.mj_step(model, data)
    viewer.render()

    i = i + 1

