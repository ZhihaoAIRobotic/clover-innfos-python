import sys


from scps.ikpykinematics import Kinematics


kin = Kinematics()

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
from scps.dynamics import gravity

pi = np.pi
pi_2 = np.pi/2

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setPositionMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))

while 1:

    arm.end_operation()

    i = 0

    u = gravity(arm.getArmPosition())

    arm.setArmTorque(u*0.1)

    print(arm.getArmPosition())



