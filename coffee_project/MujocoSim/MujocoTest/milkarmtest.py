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
Move to under steam wand
"""

sw_traj = np.load('complete_moveupsteamwand_joint.npy')

sw_q = np.zeros([len(sw_traj), 6])
sw_dq = np.zeros([len(sw_traj), 6])
sw_ddq = np.zeros([len(sw_traj), 6])

for i in range(len(sw_traj)):
    sw_q[i] = sw_traj[i, 0:6]
    sw_dq[i] = sw_traj[i, 6:12]
    sw_ddq[i] = sw_traj[i, 12:]

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
Commence trajectory tracking............................................................................................
"""

"""
Pressing the button
"""

for i in range(len(pb_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (pb_dq[i] - data.qvel) + Kp * (pb_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qpos[:] = u

    error = pb_q[i] - data.qpos
    int_er = int_er + error

    now = time.time()

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = pb_q[-1] - data.qpos

"""
Ensure the button is pressed!
"""

"""
Return to init pose after pressing button
"""

for i in range(len(apb_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (apb_dq[i] - data.qvel) + Kp * (apb_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qvel[:] = u

    error = apb_q[i] - data.qpos
    int_er = int_er + error

    now = time.time()

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = apb_q[-1] - data.qpos

while np.linalg.norm(error) > 0.08:
    u = Kp * (apb_q[-1] - data.qpos) + Ki * int_er * 0.1

    u = u * 0.6

    data.qvel[:] = u

    error = apb_q[-1] - data.qpos
    int_er = int_er + error

time.sleep(5)

"""
Move arm to under steam wand
"""

for i in range(len(sw_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (sw_dq[i] - data.qvel) + Kp * (sw_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qvel[:] = u

    error = sw_q[i] - data.qpos
    int_er = int_er + error
    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

"""
Move arm out of under the steam wand
"""

for i in range(len(sw_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (osw_dq[i] - data.qpos) + Kp * (osw_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qvel[:] = u

    error = osw_q[i] - data.qpos
    int_er = int_er + error
    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = osw_q[-1] - data.qpos

while np.linalg.norm(error) > 0.08:
    u = Kp * (osw_q[-1] - data.qpos) + Ki * int_er * 0.1

    u = u * 0.6

    data.qvel[:] = u

    error = osw_q[-1] - data.qpos
    int_er = int_er + error

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

"""
Tilt the end-effector
"""

for i in range(len(tilt_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (tilt_dq[i] - data.qvel) + Kp * (tilt_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qvel[:] = u

    error = tilt_q[i] - data.qpos
    int_er = int_er + error
    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

print("now we make art!")

"""
Commence coffee art making
"""

for i in range(len(art_q)):
    ini_t = time.time()

    theta = data.qpos
    d_theta = data.qvel

    u = Kd * (art_dq[i] - data.qvel) + Kp * (art_q[i] - data.qpos) + Ki * int_er * 0.1

    data.qvel[:] = u

    error = art_q[i] - data.qpos
    int_er = int_er + error
    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))
