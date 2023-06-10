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

arm.safePositionMode(max_vel=20 * 60, min_pos=-360, max_pos=+360)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmTorque(np.array([0, 0, 0, 0, 0, 0]))

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

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (pb_dq[i] - arm.getArmVelocity()) + Kp * (pb_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = pb_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = pb_q[-1] - arm.getArmPosition()

"""
Ensure the button is pressed!
"""

while np.linalg.norm(error) > 0.07:
    u = Kp * (pb_q[-1] - arm.getArmPosition()) + Ki * int_er * 0.1

    u = u

    arm.setArmVelocity(u)

    error = pb_q[-1] - arm.getArmPosition()
    int_er = int_er + error

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())
arm.setArmPosition([0.210, -0.32, -1.96, -2.35, 2.7, 0.02685])
time.sleep(2)
arm.setArmPosition([0.25, -0.31, -1.9, -2.3, 2.7, 0.02685])


time.sleep(4)

print("Button Pressed!")

arm.setVelocityMode()

"""
Return to init pose after pressing button
"""

for i in range(len(apb_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (apb_dq[i] - arm.getArmVelocity()) + Kp * (apb_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = apb_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

time.sleep(60)

arm.setVelocityMode()

"""
Move arm to under steam wand
"""

for i in range(len(sw_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (sw_dq[i] - arm.getArmVelocity()) + Kp * (sw_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = sw_q[i] - arm.getArmPosition()
    int_er = int_er + error
    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

time.sleep(80)

arm.setVelocityMode()
    
"""
Move arm out of under the steam wand
"""

for i in range(len(sw_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (osw_dq[i] - arm.getArmVelocity()) + Kp * (osw_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = osw_q[i] - arm.getArmPosition()
    int_er = int_er + error
    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = osw_q[-1] - arm.getArmPosition()

while np.linalg.norm(error) > 0.08:
    u = Kp * (osw_q[-1] - arm.getArmPosition()) + Ki * int_er * 0.1

    u = u * 1.2

    arm.setArmVelocity(u)

    error = osw_q[-1] - arm.getArmPosition()
    int_er = int_er + error

    # error_list.append(np.linalg.norm(error))
    # errorall.append(error)


arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

arm.setVelocityMode()

"""
Tilt the end-effector
"""

for i in range(len(tilt_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (tilt_dq[i] - arm.getArmVelocity()) + Kp * (tilt_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = tilt_q[i] - arm.getArmPosition()
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

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (art_dq[i] - arm.getArmVelocity()) + Kp * (art_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = art_q[i] - arm.getArmPosition()
    int_er = int_er + error
    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())
# arm.setArmPosition([0, 0, 0, 0, 0, 0])

"""
Plot the data!
"""

graphlist = np.array(error_list).reshape(1, np.array(error_list).shape[0])

for i in range(len(graphlist)):
    # print(row)
    plt.plot(graphlist[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

# graphlist2 = np.array(errorall).reshape(6, np.array(errorall).shape[0])
#
# for i in range(len(graphlist2)):
#     # print(row)
#     plt.plot(graphlist2[i])
#     plt.ylabel("normalized error value (meters)")
#     plt.xlabel("time_steps (0.01 seconds per step)")
#     plt.title("Error")
# plt.show()

errorall = np.array(errorall)
print(errorall.shape)
error1 = errorall[:, 0]
error2 = errorall[:, 1]
error3 = errorall[:, 2]
error4 = errorall[:, 3]
error5 = errorall[:, 4]
error6 = errorall[:, 5]
print(error1)

graphlist3 = np.array(error1).reshape(1, np.array(error1).shape[0])

for i in range(len(graphlist3)):
    # print(row)
    plt.plot(graphlist3[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

graphlist4 = np.array(error2).reshape(1, np.array(error2).shape[0])

for i in range(len(graphlist4)):
    # print(row)
    plt.plot(graphlist4[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

graphlist5 = np.array(error3).reshape(1, np.array(error3).shape[0])

for i in range(len(graphlist5)):
    # print(row)
    plt.plot(graphlist5[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

graphlist6 = np.array(error4).reshape(1, np.array(error4).shape[0])

for i in range(len(graphlist6)):
    # print(row)
    plt.plot(graphlist6[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

graphlist7 = np.array(error5).reshape(1, np.array(error5).shape[0])

for i in range(len(graphlist7)):
    # print(row)
    plt.plot(graphlist7[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

graphlist7 = np.array(error6).reshape(1, np.array(error6).shape[0])

for i in range(len(graphlist7)):
    # print(row)
    plt.plot(graphlist7[i])
    plt.ylabel("normalized error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Error")
plt.show()

# np.save("error", graphlist)