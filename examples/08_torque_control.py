import sys

import numpy as np

from scps.ikpykinematics import Kinematics

from scps.controller import PD_controller
import time

import clover_innfos_python
from clover_innfos_python import Actuator

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setCurrentMode()


input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmTorque(np.array([0, 0, 0, 0, 0, 0]))

init_time = time.time()

now_time = 0

while now_time < 3:
    arm.setCurrentMode()
    arm.setArmTorque([0, 0, 0, 0, -1, -2])

    print(arm.getArmTorque())

    now_time = time.time()-init_time

arm.setArmTorque([0, 0, 0, 0, 0, 0])
# arm.getArmVelocity([0, 0, 0, 0, 0, 0])