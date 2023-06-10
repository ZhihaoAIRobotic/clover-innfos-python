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

Kp = 0.4
Ki = 0.8
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

"""
Move arm to under coffee machine
"""

for i in range(len(ucm_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = ucm_dq[i] + Kp * (ucm_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = ucm_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())

time.sleep(2)

arm.setVelocityMode()

"""
Move arm out from under coffee machine
"""

for i in range(len(ocm_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = ocm_dq[i] + Kp * (ocm_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1

    arm.setArmVelocity(u)

    error = ocm_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())
arm.setArmPosition([0, 0, 0, 0, 0, 0])

