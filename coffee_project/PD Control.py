# PD position control test
# Only fit for tiny motions

import sys

import numpy as np
from scps.ikpykinematics import Kinematics
from scps import dynamics
kin = Kinematics()
dy = dynamics

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator


pi = np.pi
pi_2 = np.pi/2

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


# Kp = 1.5
# Kd = 1
err_inter = 0

while 1:

    jp = input()

    op = list(map(float, jp.split(',')))
    op = np.asarray(op)

    # arm.setArmPosition(op)
    error = op - arm.getArmPosition()

    theta = arm.getArmPosition()
    # print(theta)

    u = dy.gravity(theta) + Kp * (op - theta) + Kd * (0 - arm.getArmVelocity()) + Ki * err_inter * 0.02

    while abs(np.linalg.norm(error)) > 0.03:
        error = op - arm.getArmPosition()
        err_inter = err_inter + error
        arm.setVelocityMode()
        # u = dy.gravity(arm.getArmPosition()) + Kp * (op - arm.getArmPosition()) - Kd * arm.getArmVelocity()
        u = dy.gravity(arm.getArmPosition()) + Kp * (op - arm.getArmPosition()) - Kd * arm.getArmVelocity() + Ki \
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

