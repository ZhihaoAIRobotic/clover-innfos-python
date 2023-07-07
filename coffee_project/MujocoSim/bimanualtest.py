import numpy as np
import mujoco_viewer
import mujoco
import os
from scps.ikpykinematics import Kinematics
from kinematics import fkin
from coffee_project.trajectory_generation.cart_sin_traj import sin_traj
import time
from bimanualconcat import get_cof, get_milk, get_bimanual


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

"""
Milk arm trajectory
"""

# Press the button

pb_traj = np.load('buttonpressjoint.npy')

pb_q = np.zeros([len(pb_traj), 6])
pb_dq = np.zeros([len(pb_traj), 6])
pb_ddq = np.zeros([len(pb_traj), 6])

for i in range(len(pb_traj)):
    pb_q[i] = pb_traj[i, 0:6]
    pb_dq[i] = pb_traj[i, 6:12]
    pb_ddq[i] = pb_traj[i, 12:]

# Return to init pose after button pressing

apb_traj = np.load('buttonunpressjoint.npy')

apb_q = np.zeros([len(apb_traj), 6])
apb_dq = np.zeros([len(apb_traj), 6])
apb_ddq = np.zeros([len(apb_traj), 6])

for i in range(len(apb_traj)):
    apb_q[i] = apb_traj[i, 0:6]
    apb_dq[i] = apb_traj[i, 6:12]
    apb_ddq[i] = apb_traj[i, 12:]

# Move to under steam wand mid

msw_traj = np.load('milkpitchupmid.npy')

msw_q = np.zeros([len(msw_traj), 6])
msw_dq = np.zeros([len(msw_traj), 6])
msw_ddq = np.zeros([len(msw_traj), 6])

for i in range(len(msw_traj)):
    msw_q[i] = msw_traj[i, 0:6]
    msw_dq[i] = msw_traj[i, 6:12]
    msw_ddq[i] = msw_traj[i, 12:]

# Move to under steam wand top

tsw_traj = np.load('milkpitchupfull.npy')

tsw_q = np.zeros([len(tsw_traj), 6])
tsw_dq = np.zeros([len(tsw_traj), 6])
tsw_ddq = np.zeros([len(tsw_traj), 6])

for i in range(len(tsw_traj)):
    tsw_q[i] = tsw_traj[i, 0:6]
    tsw_dq[i] = tsw_traj[i, 6:12]
    tsw_ddq[i] = tsw_traj[i, 12:]

# Set up the coffee art trajectory

art_traj = np.load("art_lqt.npy")

art_q = np.zeros([len(art_traj), 6])
art_dq = np.zeros([len(art_traj), 6])
art_ddq = np.zeros([len(art_traj), 6])

for i in range(len(art_traj)):
    art_q[i] = art_traj[i, 0:6]
    art_dq[i] = art_traj[i, 6:12]
    art_ddq[i] = art_traj[i, 12:]

# Return to init pose after frothing

pitch_init_traj = np.load("pitch_to_init.npy")

pitchi_q = np.zeros([len(pitch_init_traj), 6])
pitchi_dq = np.zeros([len(pitch_init_traj), 6])
pitchi_ddq = np.zeros([len(pitch_init_traj), 6])

for i in range(len(pitch_init_traj)):
    pitchi_q[i] = pitch_init_traj[i, 0:6]
    pitchi_dq[i] = pitch_init_traj[i, 6:12]
    pitchi_ddq[i] = pitch_init_traj[i, 12:]

# Set up the art init trajectory

art_init_traj = np.load("art_init.npy")
print(art_init_traj.shape)

arti_q = np.zeros([len(art_init_traj), 6])
arti_dq = np.zeros([len(art_init_traj), 6])
arti_ddq = np.zeros([len(art_init_traj), 6])

for i in range(len(art_init_traj)):
    arti_q[i] = art_init_traj[i, 0:6]
    arti_dq[i] = art_init_traj[i, 6:12]
    arti_ddq[i] = art_init_traj[i, 12:]

"""
Coffee arm trajectory
"""

# Move to under coffee machine

ucm_traj = np.load('incomplete_movebelowcm_joint.npy')

ucm_q = np.zeros([len(ucm_traj), 6])
ucm_dq = np.zeros([len(ucm_traj), 6])
ucm_ddq = np.zeros([len(ucm_traj), 6])

for i in range(len(ucm_traj)):
    ucm_q[i] = ucm_traj[i, 0:6]
    ucm_dq[i] = ucm_traj[i, 6:12]
    ucm_ddq[i] = ucm_traj[i, 12:]

# Move out of Coffee Machine

ocm_traj = np.load('incomplete_moveoutcm_joint.npy')

ocm_q = np.zeros([len(ocm_traj), 6])
ocm_dq = np.zeros([len(ocm_traj), 6])
ocm_ddq = np.zeros([len(ocm_traj), 6])

for i in range(len(ocm_traj)):
    ocm_q[i] = ocm_traj[i, 0:6]
    ocm_dq[i] = ocm_traj[i, 6:12]
    ocm_ddq[i] = ocm_traj[i, 12:]

# Press the button

# bp_traj = np.load('complete_button_press.npy')
#
# bp_q = np.zeros([len(bp_traj), 6])
# bp_dq = np.zeros([len(bp_traj), 6])
# bp_ddq = np.zeros([len(bp_traj), 6])
#
# for i in range(len(bp_traj)):
#     bp_q[i] = bp_traj[i, 0:6]
#     bp_dq[i] = bp_traj[i, 6:12]
#     bp_ddq[i] = bp_traj[i, 12:]

# Move to design init pose

# di_traj = np.load('complete_neutral.npy')
#
# di_q = np.zeros([len(di_traj), 6])
# di_dq = np.zeros([len(di_traj), 6])
# di_ddq = np.zeros([len(di_traj), 6])
#
# for i in range(len(di_traj)):
#     di_q[i] = di_traj[i, 0:6]
#     di_dq[i] = di_traj[i, 6:12]
#     di_ddq[i] = di_traj[i, 12:]

# Design trajectory

dt_traj = np.load('complete_neutral.npy')

dt_q = np.zeros([len(dt_traj), 6])
dt_dq = np.zeros([len(dt_traj), 6])
dt_ddq = np.zeros([len(dt_traj), 6])

for i in range(len(dt_traj)):
    dt_q[i] = dt_traj[i, 0:6]
    dt_dq[i] = dt_traj[i, 6:12]
    dt_ddq[i] = dt_traj[i, 12:]

# Move to serving pose

# sp_traj = np.load('complete_neutral.npy')
#
# sp_q = np.zeros([len(sp_traj), 6])
# sp_dq = np.zeros([len(sp_traj), 6])
# sp_ddq = np.zeros([len(sp_traj), 6])
#
# for i in range(len(sp_traj)):
#     sp_q[i] = sp_traj[i, 0:6]
#     sp_dq[i] = sp_traj[i, 6:12]
#     sp_ddq[i] = sp_traj[i, 12:]

"""
Bimanual Trajectory
"""

# Get the coffee traj

# Get the coffee
getcoffee1 = get_cof(np.array([0, 0, 0, 0, 0, 0]), ucm_q)
getcoffee2 = get_milk(pb_q, ucm_q[-1])
getcoffee3 = get_milk(apb_q, ucm_q[-1])
getcoffee4 = get_milk(apb_q[-1], ocm_q)

# Get the milk froth traj
getmilkfroth1 = get_milk(msw_q, ocm_q[-1])
getmilkfroth2 = get_cof(msw_q[-1], bp_q)
getmilkfroth3 = get_cof(msw_q[-1], di_q)
getmilkfroth4 = get_milk(tsw_q, di_q[-1])

# Get to init traj
get_init = get_milk(pitchi_q, di_q[-1])

# Art time
get_art1 = get_milk(base_q, di_q[-1])
get_art2 = get_bimanual(read1_q, cofread_q[-1])
get_art3 = get_bimanual(design_q, cofdes_q)
get_art4 = get_milk(read2_q, cofdes_q[-1])
get_art5 = get_milk(fin_q, cofdes_q[-1])


"""
run the simulation
"""
i = 0

for i in range(len(q)):
    print(q[i])
    data.qpos[:] = q[i]
    # print(dq[i])
    print(">>>>>"*100)
    mujoco.mj_step(model, data)
    viewer.render()
    time.sleep(0.01)

    i = i + 1