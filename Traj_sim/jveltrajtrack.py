import sys

import numpy as np
from scps.ikpykinematics import Kinematics
from scps.controller import PD_controller
import time
import pandas as pd

import clover_innfos_python
from clover_innfos_python import Actuator

kin = Kinematics()
# con = PD_controller(0.0001, 0.0001)


pi = np.pi
pi_2 = np.pi/2

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

xcel_data = pd.read_excel('/home/clover/Github/clover-innfos-python/trajectories/test_traj/test_ilqr3d_100it_dt002.xlsx')
q_dot = pd.DataFrame(xcel_data, columns=['u1', 'u2', 'u3', 'u4', 'u5', 'u6'])
x = pd.DataFrame(xcel_data, columns=['J_dot_1', 'J_dot_2', 'J_dot_3', 'J_dot_4', 'J_dot_5', 'J_dot_6'])

x = x.to_numpy()
print(x)
print(x.shape)

input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))


i = 0

while i in range(len(x)):

    init_time = time.time()

    arm.setArmVelocity(x[i])

    print(arm.getArmPosition())

    dt = time.time() - init_time

    if dt < 0.2:
        time.sleep((0.2-dt))

    i = i+1

