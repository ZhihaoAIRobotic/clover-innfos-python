import numpy as np
import mujoco_viewer
import mujoco
import os
from scps.ikpykinematics import Kinematics
from kinematics import fkin
from coffee_project.trajectory_generation.cart_sin_traj import sin_traj
import time


kin = Kinematics()
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
# q, T, x, y = sin_traj(np.array([0, 0, -np.pi/2, -np.pi/2, np.pi/2, 0]), 1, 100)
traj = np.load("milkpitchupmid.npy")
# q = traj

q = np.zeros([len(traj), 6])
dq = np.zeros([len(traj), 6])
ddq = np.zeros([len(traj), 6])

for i in range(len(traj)):
    q[i] = traj[i, 0:6]
    # dq[i] = traj[i, 6:12]
    # ddq[i] = traj[i, 12:]

# print(ddq)

i = 0

for i in range(len(q)):
    print(q[i])
    data.qpos[:] = q[i]

    mujoco.mj_step(model, data)
    viewer.render()
    time.sleep(0.005)

    i = i + 1