import sys

import numpy as np
from scps.trajectoryconvert import traj_convert
from scps.ikpykinematics import Kinematics
from traj_circle import change
from TrajectoryCircle import TrajectoryCircle
from TrajectoryCurve import TrajectoryCurve

kin = Kinematics()

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

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setPositionMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))

while 1:

    arm.end_operation()

    i = 0

    print(arm.getArmPosition())

