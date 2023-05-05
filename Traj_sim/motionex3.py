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
from coffee_project.polynomial_trajectory import PolynomialGenerator

pg = PolynomialGenerator()

ASSETS = {}
root_path = '/home/ubuntu/Github/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/ubuntu/Github/clover-innfos-python/environment models/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

q0 = [0, 0, 0, 0, 0, 0]
dq0 = [0, 0, 0, 0, 0, 0]
ddq0 = [0, 0, 0, 0, 0, 0]
q_start = np.array([q0, dq0, ddq0])

qf = [0, np.pi / 4, -np.pi / 4, np.pi/4, 0, 0]
q_final = np.array([qf])

q, dq, ddq = pg.generate_p2p_trajectory(q_start, q_final, 10, 0, 100)

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0

for i in range(len(q)):

    data.qpos[:] = q[i]

    mujoco.mj_step(model, data)
    viewer.render()

    i = i + 1

