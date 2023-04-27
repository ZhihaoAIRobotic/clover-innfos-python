import numpy as np
import mujoco_viewer
from mujoco_py import MjSim, load_model_from_path
import mujoco
from scps.ikpykinematics import Kinematics
import matplotlib.pyplot as plt
import os
from scps.dynamics import gravity, inertia, centrifugalterms
from scps.controller import PD_controller
from scps.ilqt import ilqt


ASSETS = {}
root_path = '/home/ubuntu/Github/clover-innfos-python/environment models/meshes'
files = os.listdir(root_path)
for file in files:
    f = open(os.path.join(root_path, file), 'rb')
    num = file.split('_')[0]
    ASSETS['{}_Link.stl'.format(num)] = f.read()

model = mujoco.MjModel.from_xml_path("/home/ubuntu/Github/clover-innfos-python/environment models/gluon_robot.xml", ASSETS)
data = mujoco.MjData(model)

kin = Kinematics()
con = PD_controller()
con.kd = 50
con.kp = 50

Target = np.array([0.2, -0.1, -0.1, 0.2, 0.3, 0.2])
# Traj = np.array([[0, 0, 0, 0, 0, 0],
#                  [-0.10808229, 5.93054473, 83.56811523, 3.47045898, 0.09765625, 86.52282715],
#                  [-4.94830012, 15.38734794, 134.1973877, 62.75939941, -0.73364258, 86.5222168],
#                  [-3.97674561, 50.89657676, 120.01220703, 87.05993652, -7.04406738, 86.52160645],
#                  [-4.89110529, 60.83504009, 95.49743652, 77.04040527, -2.04101562, 86.52282715],
#                  [3.00322831, 73.76044106, 66.89868164, 49.23095703, 2.04040527, 86.52099609]])

# traj = Traj*(np.pi/180)
# traj = ilqt(traj)

viewer = mujoco_viewer.MujocoViewer(model, data)

while data.qpos[2] != Target[2]:

    theta = data.qpos[:]
    theta_dot = data.qvel[:]

    u = con.pos_control(theta, Target, theta_dot)

    data.qvel[:] = u

    print(data.qpos[:])

    print("-"*80)

    mujoco.mj_step(model, data)
    viewer.render()

