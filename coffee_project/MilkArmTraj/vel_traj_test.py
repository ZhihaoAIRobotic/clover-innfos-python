# PID control for a set of via points for coffee art

from scps.ikpykinematics import Kinematics
from scps import dynamics
from coffee_project.trajectory_generation.cart_sin_traj import sin_traj
from kinematics import jacobian
import matplotlib.pyplot as plt

kin = Kinematics()
dy = dynamics

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator

pi = np.pi
pi_2 = np.pi / 2

"""
Program initiation
"""

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Vel)

input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=1 * 60, min_pos=-360, max_pos=+360)
arm.safeVelocityMode(max_acc=400, max_dec=200, max_vel=10 * 60)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmTorque(np.array([0, 0, 0, 0, 0, 0]))

Kp = 2.9
Ki = 2.7
Kd = 1.2
int_er = 0

error_list = []
errorall = []

"""
Press the button
"""

pb_traj = np.load('buttonpressjoint.npy')

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

apb_traj = np.load('buttonunpressjoint.npy')

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

# """
# Set up the coffee art trajectory
# """
#
# art_traj = np.load("coffee_art_pose_fix_10sec.npy")
#
# art_q = np.zeros([len(art_traj), 6])
# art_dq = np.zeros([len(art_traj), 6])
# art_ddq = np.zeros([len(art_traj), 6])
#
# for i in range(len(art_traj)):
#     art_q[i] = art_traj[i, 0:6]
#     art_dq[i] = art_traj[i, 6:12]
#     art_ddq[i] = art_traj[i, 12:]
#
"""
Set up the coffee art trajectory
"""

art_traj = np.load("art_lqt.npy")

art_q = np.zeros([len(art_traj), 6])
art_dq = np.zeros([len(art_traj), 6])
art_ddq = np.zeros([len(art_traj), 6])

for i in range(len(art_traj)):
    art_q[i] = art_traj[i, 0:6]
    art_dq[i] = art_traj[i, 6:12]
    art_ddq[i] = art_traj[i, 12:]

"""
Return to init pose after frothing
"""

pitch_init_traj = np.load("pitch_to_init.npy")

pitchi_q = np.zeros([len(pitch_init_traj), 6])
pitchi_dq = np.zeros([len(pitch_init_traj), 6])
pitchi_ddq = np.zeros([len(pitch_init_traj), 6])

for i in range(len(pitch_init_traj)):
    pitchi_q[i] = pitch_init_traj[i, 0:6]
    pitchi_dq[i] = pitch_init_traj[i, 6:12]
    pitchi_ddq[i] = pitch_init_traj[i, 12:]

"""
Set up the art init trajectory
"""

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
Get pitcher to just surface touching the wand
"""

for i in range(len(msw_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (msw_dq[i] - arm.getArmVelocity()) + Kp * (msw_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    if any(abs(q) > 4.5 for q in u):
        print("uh oh, too fast")
        arm.setPositionMode()
        arm.setArmPosition(np.array(arm.getArmPosition()))

    arm.setArmVelocity(u)

    error = msw_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print(u)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

arm.setArmPosition([0.4853, 0.08625, -1.905, 1.305, -2.517, 3.1])

time.sleep(4)

arm.setVelocityMode()

"""
Get pitcher to under surface touching the wand
"""

for i in range(len(tsw_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (tsw_dq[i] - arm.getArmVelocity()) + Kp * (tsw_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    if any(abs(q) > 4.5 for q in u):
        print("......" * 100)
        print("uh oh, too fast")
        arm.setPositionMode()
        arm.setArmPosition(np.array(arm.getArmPosition()))

    arm.setArmVelocity(u)

    error = tsw_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))
    print(u)

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

time.sleep(5)

arm.setVelocityMode()

"""
Pitch to init
"""

for i in range(len(pitchi_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (pitchi_dq[i] - arm.getArmVelocity()) + Kp * (pitchi_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    if any(abs(q) > 5.5 for q in u):
        print("......" * 100)
        print("uh oh, too fast")
        arm.setPositionMode()
        arm.setArmPosition(np.array(arm.getArmPosition()))

    arm.setArmVelocity(u)

    error = pitchi_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))
    print(u)

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())
time.sleep(1)
arm.setArmPosition([0, 0, -2.356194, -2.356194, 1.570796, 0])
print("****" * 100)
print("Time to make art!")
print("****" * 100)

arm.setVelocityMode()

"""
Make Coffee Art
"""

for i in range(len(art_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (art_dq[i] - arm.getArmVelocity()) + Kp * (art_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1
    if i == 2:
        u = u * 0.8

    if i == 3:
        u = u * 0.8

    if any(abs(q) > 4.5 for q in u):
        print("......" * 100)
        print(arm.getArmPosition())
        print(art_q[i])
        print(art_dq[i])
        print("uh oh, too fast")
        arm.setPositionMode()
        arm.setArmPosition(np.array(arm.getArmPosition()))

    arm.setArmVelocity(u)

    error = art_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))
    print(u)

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

graphlist = np.array(error_list).reshape(1, np.array(error_list).shape[0])

for i in range(len(graphlist)):
    # print(row)
    plt.plot(graphlist[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

np.save("error", graphlist)