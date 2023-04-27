import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
from Kinematics import kinematics


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
    lastpos = arm.getArmPosition()

    DH_parameters = {
        "": ['d', 'θ', 'a', 'α'],
        1: [0.105, pi_2, 0.0, pi_2],
        2: [0.0, pi_2, 0.17447, 0.0],
        3: [0.0, 0.0, 0.17442, pi],
        4: [-0.080113, pi_2, 0.0, pi_2],
        5: [0.080089, 0.0, 0.0, pi_2],
        6: [0.0, 0.0, 0.0, 0.0],
    }

    robot_chain = [kinematics.DH(*DH_parameters[i]) for i in range(1, 7)]

    Transform = kinematics.fk(robot_chain, lastpos)

    # print(Transform)
    print("Joint Mode?")
    jm = input()

    if jm == 'Joint Position':

        jp = input()

        # print(np.array([jv]))

        op = list(map(float, jp.split(',')))
        op = np.asarray(op)

        # op = np.asarray(op)

        op[:] = op[:]*180/pi
        print(op)
        op[0] = -op[0]
        op[1] = -op[1]
        op[4] = -op[4]
        op[5] = -op[5]
        print(op)

        arm.setArmPosition(op)

        curpos = arm.getArmPosition()

        T = kinematics.fk(robot_chain, curpos)

        print(T)

    if jm == 'Joint Velocity':

        eepos = arm.getArmPosition()

        jv = input()

        i = 0

        while i < 100:

        # print(np.array([jv]))

            op = list(map(float, jv.split(',')))
            op = np.asarray(op)

        # op = np.asarray(op)

            op[:] = op[:] * 18 / pi
            # print(op)
            op[0] = -op[0]
            op[1] = -op[1]
            op[4] = -op[4]
            op[5] = -op[5]
            # print(op)            now = arm.getArmPosition() + op

            now = arm.getArmPosition() + op

            if (arm.getArmPosition() != eepos).all():

                op = 0

            arm.setArmPosition(now)

            print(now)

            i = i + 1











