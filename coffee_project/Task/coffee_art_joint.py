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
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))

Kp = 0.4
Ki = 0.8
int_er = 0

"""
Set up the coffee art init pose
"""

init_traj = np.load('complete_init.npy')

init_q = np.zeros([len(init_traj), 6])
init_dq = np.zeros([len(init_traj), 6])
init_ddq = np.zeros([len(init_traj), 6])

for i in range(len(init_traj)):
    init_q[i] = init_traj[i, 0:6]
    init_dq[i] = init_traj[i, 6:12]
    init_ddq[i] = init_traj[i, 12:]

"""
Move arm to correct pouring pose
"""

for i in range(len(init_q)):
    ini_t = time.time()

    theta = arm.getArmPosition()
    d_theta = arm.getArmVelocity()

    u = init_dq[i] + Kp * (init_q[i] - arm.getArmPosition()) + Ki * int_er * 0.1
    #
    # dy.gravity(arm.getArmPosition())*1.6 + Kp * (init_q[i] - arm.getArmPosition()) + Kd * (
    #         init_dq[i] - arm.getArmVelocity())

    arm.setArmVelocity(u)

    error = init_q[i] - arm.getArmPosition()
    int_er = int_er + error

    now = time.time()

    print("Time Elapsed: " + str(now - ini_t))
    print(error)
    print(init_dq[i])
    print(arm.getArmPosition())

arm.setPositionMode()
arm.setArmPosition(arm.getArmPosition())
arm.setArmPosition([0, 0, 0, 0, 0, 0])

print(init_q[-1])
print(arm.getArmPosition())
# arm.disableAllActuators()