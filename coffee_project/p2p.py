# Point to Point Coffee Making

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
import matplotlib.pyplot as plt


pi = np.pi
pi_2 = np.pi/2
Kp = 0.5

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
# arm.setPositionMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
errorlist = []

while 1:

    jp = input()

    op = list(map(float, jp.split(',')))
    op = np.asarray(op)

    print(op)

    arm.setArmPosition(op)

    error = op - arm.getArmPosition()
    errorlist.append(error)

    q = arm.getArmPosition()
    theta = q[:] + Kp*(error)
    arm.setArmPosition(theta)

    for row in errorlist:
        plt.plot(row)
        plt.ylabel("θ (rads)")
        plt.xlabel("Iterations")
        plt.title("Error iterations")
    plt.show()