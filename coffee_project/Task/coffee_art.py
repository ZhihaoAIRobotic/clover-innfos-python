# PID control for a set of via points for coffee art

from scps.ikpykinematics import Kinematics
from scps import dynamics
from coffee_project.trajectory_generation.cart_sin_traj import sin_traj
from kinematics import jacobian

kin = Kinematics()
dy = dynamics

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator

pi = np.pi
pi_2 = np.pi / 2

"""
Program initiation
"""

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)

input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=10 * 60, min_pos=-360, max_pos=+360)
# arm.set_safety_values()
# arm.setPositionMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmVelocity(np.array([0, 0, 0, 0, 0, 0]))
arm.setArmTorque(np.array([0, 0, 0, 0, 0, 0]))

Kp = 1.6
Kd = 0.6
Ki = 0.5
err_inter = 0

print("..."*100)
"""
Move to correct angle to pour milk
"""

init_pose = np.array([-0.000887, -0.03569, -1.563, -0.8339, 1.712, 0])

error = init_pose - arm.getArmPosition()

theta = arm.getArmPosition()

u = dy.gravity(theta) + Kp * (init_pose - theta) + Kd * (0 - arm.getArmVelocity()) + Ki * err_inter * 0.02

while abs(np.linalg.norm(error)) > 0.07:
    error = init_pose - arm.getArmPosition()
    err_inter = err_inter + error
    arm.setVelocityMode()
    # u = dy.gravity(arm.getArmPosition()) + Kp * (op - arm.getArmPosition()) - Kd * arm.getArmVelocity()
    u = dy.gravity(arm.getArmPosition()) + Kp * (init_pose - arm.getArmPosition()) - Kd * arm.getArmVelocity() + Ki \
        * err_inter * 0.02

    arm.setArmVelocity(u)

    err_inter = err_inter + error

print("Time to make art")

"""
Commence coffee art making
"""

traj = np.load('coffee_traj_dt04_6by1.npy')

x = np.zeros([len(traj), 6])
dx = np.zeros([len(traj), 6])
ddx = np.zeros([len(traj), 6])

for i in range(len(traj)):
    x[i] = traj[i, 0:6]
    dx[i] = traj[i, 6:12]
    ddx[i] = traj[i, 12:]

Kp = 100
Kd = 10

arm.setCurrentMode()

for i in range(len(x)):
    # First Step
    init = time.time()
    q = arm.getArmPosition()
    dq = arm.getArmVelocity()

    J = jacobian.get_J(q)
    dJ = jacobian.get_dJ(q, dq)

    p = kin.fk_to_pose(q)
    dp = np.matmul(dq, J)

    y = np.matmul(np.linalg.inv(J), (ddx[i] + Kd * (dx[i] - dp) + Kp * (x[i] - p) - np.matmul(dJ, dp)))
    u = np.matmul(dy.inertia(q), y) + dy.centrifugalterms(q, dq) + dy.gravity(q)
    arm.setArmTorque(u)

    # # Error Term
    # q = arm.getArmPosition()
    # p = kin.fk_to_pose(q)
    # error = x[i] - p
    #
    # while abs(np.linalg.norm(error)) > 0.07:
    #     arm.setCurrentMode()
    #
    #     q = arm.getArmPosition()
    #     dq = arm.getArmVelocity()
    #
    #     J = jacobian.get_J(q)
    #     dJ = jacobian.get_dJ(q, dq)
    #
    #     p = kin.fk_to_pose(q)
    #     dp = np.matmul(dq, J)
    #
    #     y = np.matmul(np.linalg.inv(J), (ddx[i] + Kd * (dx[i] - dp) + Kp * (x[i] - p) - np.matmul(dJ, dp)))
    #     u = np.matmul(dy.inertia(q), y) + dy.centrifugalterms(q, dq) + dy.gravity(q)
    #
    #     arm.setArmVelocity(u)
    #
    #     # Error Term Again
    #     q = arm.getArmPosition()
    #     p = kin.fk_to_pose(q)
    #     error = x[i] - p

    now = time.time()
    print("Time Elapsed: " + str(now - init))
    print(arm.getArmTorque())



