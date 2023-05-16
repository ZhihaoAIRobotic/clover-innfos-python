import numpy as np
import mujoco_viewer
# from mujoco_py import MjSim, load_model_from_path
import mujoco
import matplotlib.pyplot as plt
import os
from kinematics import fk
from scps.dynamics import gravity, inertia, centrifugalterms
from scps.controller import PD_controller
import pandas as pd
import time


ASSETS = {}
root_path = '/home/hengyi/GitHub/clover-innfos-python/Urdf/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/hengyi/GitHub/clover-innfos-python/Urdf/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

i = 0

for i in range(10):

    data.qpos = [-np.pi/2, 0, np.pi/2, 0, np.pi/2, 0]
    print(fk([-np.pi/2, 0, np.pi/2, 0, np.pi/2, 0]))

    mujoco.mj_step(model, data)
    viewer.render()
    time.sleep(5)

    i = i + 1








