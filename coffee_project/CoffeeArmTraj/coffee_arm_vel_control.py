# PID control for a set of via points for coffee art

from scps.ikpykinematics import Kinematics
from scps import dynamics
from coffee_project.trajectory_generation.cart_sin_traj import sin_traj
from kinematics import jacobian

kin = Kinematics()
dy = dynamics

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
import matplotlib.pyplot as plt

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

arm.safePositionMode(max_vel=10 * 60, min_pos=-360, max_pos=+360)
# arm.set_safety_values()
# arm.setPositionMode()
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
Move to under coffee machine
"""

ucm_traj = np.load('incomplete_movebelowcm_joint.npy')

ucm_q = np.zeros([len(ucm_traj), 6])
ucm_dq = np.zeros([len(ucm_traj), 6])
ucm_ddq = np.zeros([len(ucm_traj), 6])

for i in range(len(ucm_traj)):
    ucm_q[i] = ucm_traj[i, 0:6]
    ucm_dq[i] = ucm_traj[i, 6:12]
    ucm_ddq[i] = ucm_traj[i, 12:]
    
"""
Move out of Coffee Machine
"""

ocm_traj = np.load('incomplete_moveoutcm_joint.npy')

ocm_q = np.zeros([len(ocm_traj), 6])
ocm_dq = np.zeros([len(ocm_traj), 6])
ocm_ddq = np.zeros([len(ocm_traj), 6])

for i in range(len(ocm_traj)):
    ocm_q[i] = ocm_traj[i, 0:6]
    ocm_dq[i] = ocm_traj[i, 6:12]
    ocm_ddq[i] = ocm_traj[i, 12:]
# 
# """
# Press the button
# """
# 
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
# 
# """
# Move to pouring pose
# """
# 
# pp_traj = np.load('complete_neutral.npy')
# 
# pp_q = np.zeros([len(pp_traj), 6])
# pp_dq = np.zeros([len(pp_traj), 6])
# pp_ddq = np.zeros([len(pp_traj), 6])
# 
# for i in range(len(pp_traj)):
#     pp_q[i] = pp_traj[i, 0:6]
#     pp_dq[i] = pp_traj[i, 6:12]
#     pp_ddq[i] = pp_traj[i, 12:]
# 
# """
# Move to serving pose
# """
# 
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

error_list = []
errorall = []

"""
Move arm to under coffee machine
"""

for i in range(len(ucm_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (ucm_dq[i] - arm.getArmVelocity()) + Kp * (ucm_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = ucm_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))
    print(np.linalg.norm(error))

error = ucm_q[-1] - arm.getArmPosition()

while np.linalg.norm(error) > 0.06:
    u = Kp * (ucm_q[-1] - arm.getArmPosition()) + Ki * int_er * 0.1

    u = u * 0.6

    arm.setArmVelocity(u)

    error = ucm_q[-1] - arm.getArmPosition()
    int_er = int_er + error

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

time.sleep(5)

arm.setVelocityMode()

"""
Move arm out from under coffee machine
"""

for i in range(len(ocm_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = Kd * (ocm_dq[i] - arm.getArmVelocity()) + Kp * (ocm_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = ocm_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    error_list.append(np.linalg.norm(error))
    errorall.append(error)

    print("Time Elapsed: " + str(now - ini_t))

error = ocm_q[-1] - arm.getArmPosition()

while np.linalg.norm(error) > 0.06:
    u = Kp * (ucm_q[-1] - arm.getArmPosition()) + Ki * int_er * 0.1

    u = u * 0.6

    arm.setArmVelocity(u)

    error = ocm_q[-1] - arm.getArmPosition()
    int_er = int_er + error

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
    plt.title("Normalized Error")
plt.show()

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
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 1 error")
plt.show()

graphlist4 = np.array(error2).reshape(1, np.array(error2).shape[0])

for i in range(len(graphlist4)):
    # print(row)
    plt.plot(graphlist4[i])
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 2 error")
plt.show()

graphlist5 = np.array(error3).reshape(1, np.array(error3).shape[0])

for i in range(len(graphlist5)):
    # print(row)
    plt.plot(graphlist5[i])
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 3 error")
plt.show()

graphlist6 = np.array(error4).reshape(1, np.array(error4).shape[0])

for i in range(len(graphlist6)):
    # print(row)
    plt.plot(graphlist6[i])
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 4 error")
plt.show()

graphlist7 = np.array(error5).reshape(1, np.array(error5).shape[0])

for i in range(len(graphlist7)):
    # print(row)
    plt.plot(graphlist7[i])
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 5 error")
plt.show()

graphlist8 = np.array(error6).reshape(1, np.array(error6).shape[0])

for i in range(len(graphlist8)):
    # print(row)
    plt.plot(graphlist8[i])
    plt.ylabel("error value (meters)")
    plt.xlabel("time_steps (0.01 seconds per step)")
    plt.title("Joint 6 error")
plt.show()

average_error1 = np.average(error1)
average_error2 = np.average(error2)
average_error3 = np.average(error3)
average_error4 = np.average(error4)
average_error5 = np.average(error5)
average_error6 = np.average(error6)

average_errornorm = np.average(error_list)

print(average_error1)
print(average_error2)
print(average_error3)
print(average_error4)
print(average_error5)
print(average_error6)
print(average_errornorm)

