# Trajectory Tracking for Coffee Making

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
import matplotlib.pyplot as plt
from scps import dynamics, ikpykinematics
from kinematics import jacobian, fkin
from scipy.spatial.transform import Rotation as R

kin = ikpykinematics.Kinematics()
dy = dynamics

pi = np.pi
pi_2 = np.pi/2
Kp = 0.5
Kd = 0.5

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
errorlist = []
Kp = 1000
Kd = 1000

traj = np.load("/coffee_project/Test Code/test_traj.npy")
x = traj[:, 0:7]
dx = traj[:, 7:14]
ddx = traj[:, 14:21]

while 1:
    for i in range(len(dx)):
        q = arm.getArmPosition()
        dq = arm.getArmVelocity()
        J = jacobian.get_J(q)
        fx = fkin.fk_pose(q)
        dfx = jacobian.get_dJ(q, dq) @ dq

        x_e = x[i] - fx

        u = dy.gravity(q) + J.T * Kp * x_e[:6] - J.T * Kd * dfx
        # print(u)

        arm.setArmVelocity(u)

        errorlist.append(x_e)

    arm.setArmVelocity(np.zeros(6))

print(errorlist)