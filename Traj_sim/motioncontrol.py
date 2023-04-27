import sys

import numpy as np
from scps.trajectoryconvert import traj_convert
from scps.ikpykinematics import Kinematics
from traj_circle import change
from TrajectoryCircle import TrajectoryCircle
from TrajectoryCurve import TrajectoryCurve
from scps.controller import PD_controller

kin = Kinematics()
con = PD_controller(0.0001, 0.0001)

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

# motion = np.load('/home/clover/Github/Rofunc/rofunc/data/taichi_1l.npy')
# motion = motion*0.0001
# q = traj_convert(motion)

# q = np.array([[0,0,0,0,0,0],
#               [0,0.1,-0.1,0,0,0],
#               [0,0.2,-0.2,0,0,0],
#               [0, 0.3, -0.3, 0, 0, 0],
#               [0,0.4, -0.4, 0,0,0],
#               [0, 0.5, -0.5, 0, 0, 0],
#               [0,0.6, -0.6, 0,0,0],
#               [0,0.7, -0.7, 0,0,0],
#               [0,0.8, -0.8, 0,0,0],
#               [0,0.9, -0.9, 0,0,0],
#               [0,1.5, -1.5, 0,0,0],
#               [0,0.4, -0.4, 0,0,0],
#               [0,0, 0, 0,0,0]])

# traj = TrajectoryCircle(150, x_axis=True, y_axis=False, z_axis=False, orientation='None')
# a = traj.get_instant_circle(0.1)
# b = traj.get_instant_rotation()

Target = np.array(
    [-0.19190641732275654, -2.661618746277845, -1.7225457138037745, 0.9390738728429078, 0.19190412669933546,
     -8.201720369527266e-07])

Rotation = ([1, 0, 0],
            [0, 1, 0],
            [0, 0, 1])

c = TrajectoryCurve(Target, 100, Rotation, orientation=None)
a = c.get_instant_curve()
b = c.get_instant_rotation()


q = change(a, b)

while 1:

    for i in range(len(q)):
        # op = q[i]
        op = q[i][1:7]
        print(op)
        op[0] = -op[0]
        op[1] = -op[1]
        op[4] = -op[4]
        op[5] = -op[5]

        op[:] = op[:] * 180 / pi
        arm.setArmPosition(op)

        while arm.getArmPosition() != op:
            jv = con.pos_control(arm.getArmPosition(), op, arm.getArmVelocity())

            jv[:] = jv[:] * 180 / pi
            arm.setArmPosition(jv)

            print(arm.getArmPosition())

        time.sleep(0.1)

        i = i+1

        if i == len(q):

            arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))

            sys.exit()