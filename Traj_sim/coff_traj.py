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

"""coffee_project Trajectory"""

q = np.array([[0,0,0,0,0,0],
              [0, 9.164, 113.68, 0, -0.216, 112.94],
              [0.018, -4.419, 136.04, 2.131, -0.004, 142.49],
              [0.4940, 63.507, 69.164, 1.664, 2.498, 123.50],
              [0.018, -4.419, 136.04, 2.131, -0.004, 142.49],
              [0,0,0,0,0,0]])

while 1:

    i = 0

    for i in range(len(q)):
        op = q[i]
        print(op)
        op[0] = -op[0]
        op[1] = -op[1]
        op[4] = -op[4]
        op[5] = op[5]

        arm.setArmPosition(op)


        time.sleep(1.5)

        i = i+1

        if i == len(q):

            arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))

            sys.exit()