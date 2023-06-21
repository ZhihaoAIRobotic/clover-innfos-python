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

Kp = 2.9
Ki = 2.7
Kd = 1.2
int_er = 0

"""
Press the button
"""

pb_traj = np.load('complete_mkpressbutton_joint.npy')

pb_q = np.zeros([len(pb_traj), 6])
pb_dq = np.zeros([len(pb_traj), 6])
pb_ddq = np.zeros([len(pb_traj), 6])

for i in range(len(pb_traj)):
    pb_q[i] = pb_traj[i, 0:6]
    pb_dq[i] = pb_traj[i, 6:12]
    pb_ddq[i] = pb_traj[i, 12:]

"""
Return to init pose after button pressing
"""

apb_traj = np.load('complete_mkafterpressbutton_joint.npy')

apb_q = np.zeros([len(apb_traj), 6])
apb_dq = np.zeros([len(apb_traj), 6])
apb_ddq = np.zeros([len(apb_traj), 6])

for i in range(len(apb_traj)):
    apb_q[i] = apb_traj[i, 0:6]
    apb_dq[i] = apb_traj[i, 6:12]
    apb_ddq[i] = apb_traj[i, 12:]

"""
Move to under steam wand mid
"""

msw_traj = np.load('milkpitchupmid.npy')

msw_q = np.zeros([len(msw_traj), 6])
msw_dq = np.zeros([len(msw_traj), 6])
msw_ddq = np.zeros([len(msw_traj), 6])

for i in range(len(msw_traj)):
    msw_q[i] = msw_traj[i, 0:6]
    msw_dq[i] = msw_traj[i, 6:12]
    msw_ddq[i] = msw_traj[i, 12:]

"""
Move to under steam wand top
"""

tsw_traj = np.load('milkpitchupfull.npy')

tsw_q = np.zeros([len(tsw_traj), 6])
tsw_dq = np.zeros([len(tsw_traj), 6])
tsw_ddq = np.zeros([len(tsw_traj), 6])

for i in range(len(tsw_traj)):
    tsw_q[i] = tsw_traj[i, 0:6]
    tsw_dq[i] = tsw_traj[i, 6:12]
    tsw_ddq[i] = tsw_traj[i, 12:]

"""
Move out of steam wand
"""

osw_traj = np.load('complete_movedownsteamwand_joint.npy')

osw_q = np.zeros([len(osw_traj), 6])
osw_dq = np.zeros([len(osw_traj), 6])
osw_ddq = np.zeros([len(osw_traj), 6])

for i in range(len(osw_traj)):
    osw_q[i] = osw_traj[i, 0:6]
    osw_dq[i] = osw_traj[i, 6:12]
    osw_ddq[i] = osw_traj[i, 12:]

"""
Set up the milk jug tilt to pour pose
"""

tilt_traj = np.load('complete_tilt.npy')

tilt_q = np.zeros([len(tilt_traj), 6])
tilt_dq = np.zeros([len(tilt_traj), 6])
tilt_ddq = np.zeros([len(tilt_traj), 6])

for i in range(len(tilt_traj)):
    tilt_q[i] = tilt_traj[i, 0:6]
    tilt_dq[i] = tilt_traj[i, 6:12]
    tilt_ddq[i] = tilt_traj[i, 12:]

"""
Set up the coffee art trajectory
"""

art_traj = np.load("coffee_art_pose_fix_10sec.npy")

art_q = np.zeros([len(art_traj), 6])
art_dq = np.zeros([len(art_traj), 6])
art_ddq = np.zeros([len(art_traj), 6])

for i in range(len(art_traj)):
    art_q[i] = art_traj[i, 0:6]
    art_dq[i] = art_traj[i, 6:12]
    art_ddq[i] = art_traj[i, 12:]

error_list = []
errorall = []

"""
Commence trajectory tracking Simulation.................................................................................
"""

while 1:
    for i in range(len(msw_q)):
        data.qpos[:] = msw_q[i]

        mujoco.mj_step(model, data)
        viewer.render()
        time.sleep(0.01)

    for i in range(len(tsw_dq)):
        data.qpos[:] = tsw_q[i]

        mujoco.mj_step(model, data)
        viewer.render()
        time.sleep(0.01)