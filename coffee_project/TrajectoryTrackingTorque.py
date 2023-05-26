# Trajectory Tracking for Coffee Making

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
import matplotlib.pyplot as plt
from coffee_project.polynomial_trajectory import PolynomialGenerator
from coffee_project.polynomial_trajectory_cartesian import PolynomialGeneratorCart
from scps import dynamics
from kinematics import jacobian, fkin

pg = PolynomialGenerator()
pgc = PolynomialGeneratorCart()
dy = dynamics

pi = np.pi
pi_2 = np.pi/2
Kp = 0.5
Kd = 0.5

actuator_ids = [1, 2, 3, 4, 5, 6]
arm = clover_innfos_python.ArmInterface()
arm.enableAllActuators()

arm.activateActuatorModeInBantch(arm.jointlist, Actuator.ActuatorMode.Mode_Cur)


input("Move to zero")
arm.home()  # Will set position to profile mode

arm.safePositionMode(max_vel=30 * 60, min_pos=-360, max_pos=+360)
arm.setVelocityMode()

input("Ready")

arm.setArmPosition(np.array([0, 0, 0, 0, 0, 0]))
errorlist = []

traj = np.load("/coffee_project/Test Code/test_traj.npy")
x = traj[:, 0:7]
dx = traj[:, 7:14]
ddx = traj[:, 14:21]

while 1:
    for i in range(len(x)):
        q = arm.getArmPosition()
        dq = arm.getArmVelocity()
        J = jacobian.get_J(q)
        dJ = jacobian.get_dJ(q, dq)
        fx = fkin.fk_pose(q)
        dfx = jacobian.get_dJ(q, dq) @ dq

        x_e = x - fx
        dx_e = dx - dfx

        y = np.linalg.inv(J) * (ddx + Kd * dx_e + Kp * x_e - dJ @ dq)
        u = dy.inertia(q) @ y + dy.centrifugalterms(q, dq) + dy.gravity(q)

        errorlist.append(x_e)

        arm.setArmTorque(u)
    arm.setArmTorque(np.zeros(6))
