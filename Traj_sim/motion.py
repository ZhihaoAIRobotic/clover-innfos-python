import numpy as np
from motion_and_traj import trajectoryimport
from scps.ikpykinematics import Kinematics

kin = Kinematics()
traj = trajectoryimport.traj_import()

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
    # print(Transform)
    print("Joint Mode?")

    for i in len(traj):

        op = traj[i]
        op[:] = op[:] * 180 / pi
        print(op)
        op[0] = -op[0]
        op[1] = -op[1]
        op[4] = -op[4]
        op[5] = -op[5]
        print(op)

        arm.setArmPosition(op)


        time.sleep(0.1)


    # jm = input()
    #
    # if jm == 'Joint Position':
    #
    #     jp = input()
    #
    #     # print(np.array([jv]))
    #
    #     op = list(map(float, jp.split(',')))
    #     op = np.asarray(op)
    #
    #     # op = np.asarray(op)
    #
    #     op[:] = op[:]*180/pi
    #     print(op)
    #     op[0] = -op[0]
    #     op[1] = -op[1]
    #     op[4] = -op[4]
    #     op[5] = -op[5]
    #     print(op)
    #
    #     arm.setArmPosition(op)
    #
    #     curpos = arm.getArmPosition()
    #
    #     T = kin.fk(robot_chain, curpos)
    #
    # if jm == 'Joint Velocity':
    #
    #     eepos = arm.getArmPosition()
    #
    #     jv = input()
    #
    #     i = 0
    #
    #     while i < 100:
    #
    #     # print(np.array([jv]))
    #
    #         op = list(map(float, jv.split(',')))
    #         op = np.asarray(op)
    #
    #     # op = np.asarray(op)
    #
    #         op[:] = op[:] * 18 / pi
    #         # print(op)
    #         op[0] = -op[0]
    #         op[1] = -op[1]
    #         op[4] = -op[4]
    #         op[5] = -op[5]
    #         # print(op)            now = arm.getArmPosition() + op
    #
    #         now = arm.getArmPosition() + op
    #
    #         if (arm.getArmPosition() != eepos).all():
    #
    #             op = 0
    #
    #         arm.setArmPosition(now)
    #
    #         print(now)
    #
    #         i = i + 1