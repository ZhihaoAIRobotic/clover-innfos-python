# PID control for a set of via points for coffee art

import sys

import numpy as np
from scps.ikpykinematics import Kinematics
from scps import dynamics
from coffee_project.cart_sin_traj import sin_traj

kin = Kinematics()
dy = dynamics

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator

pi = np.pi
pi_2 = np.pi / 2

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=10 * 60, min_pos=-360, max_pos=+360)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))

Kp = 1.7
Kd = 0.6
Ki = 0.6
err_inter = 0

q, T, x, y = sin_traj(np.array([0, 0, -np.pi/2, -np.pi/2, np.pi/2, 0]), 1, 100)

while 1:

    for i in range(len(q)):

        # arm.setArmPosition(op)
        error = q[i] - arm.getArmPosition()

        theta = arm.getArmPosition()
        # print(theta)

        u = dy.gravity(theta) + Kp * (q[i] - theta) + Kd * (0 - arm.getArmVelocity()) + Ki * err_inter * 0.02

        while abs(np.linalg.norm(error)) > 0.03:
            error = q[i] - arm.getArmPosition()
            err_inter = err_inter + error
            arm.setVelocityMode()
            # u = dy.gravity(arm.getArmPosition()) + Kp * (op - arm.getArmPosition()) - Kd * arm.getArmVelocity()
            u = dy.gravity(arm.getArmPosition()) + Kp * (q[i] - arm.getArmPosition()) - Kd * arm.getArmVelocity() + Ki \
                * err_inter * 0.02

            arm.setArmVelocity(u)

            print("....." * 80)
            # print(dy.gravity(arm.getArmPosition()))
            # print((Kp * (op - arm.getArmPosition()) - Kd * arm.getArmVelocity()))

            err_inter = err_inter + error

            print(np.linalg.norm(error))
            print(error[2])

            # print(arm.getArmPosition())

        print(arm.getArmPosition())
        arm.setArmVelocity([0, 0, 0, 0, 0, 0])

        arm.setPositionMode()
        arm.setArmPosition(arm.getArmPosition())