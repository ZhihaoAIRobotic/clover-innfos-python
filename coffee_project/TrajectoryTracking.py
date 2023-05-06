# Trajectory Tracking for Coffee Making

import time

import numpy as np
import clover_innfos_python
from clover_innfos_python import Actuator
import matplotlib.pyplot as plt
from coffee_project.polynomial_trajectory import PolynomialGenerator
from scps import dynamics


pg = PolynomialGenerator()
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
# errorlist = []

q0 = [0, 0, 0, 0, 0, 0]
dq0 = [0, 0, 0, 0, 0, 0]
ddq0 = [0, 0, 0, 0, 0, 0]
q_start = np.array([q0, dq0, ddq0])

via_points = np.array([[0, np.pi/4, -np.pi/4, 0, 1.1, 0.2],
                       [0, np.pi/2, -np.pi/2, 0, 0.2, 0.5],
                       [0, 0.3, 0.2, 0.1, 1.1, 0.2],
                       [0, 0, 0, 0, 0, 0]])
t_list = np.array([10.0, 15.0, 20.0, 21.0])
n_list = np.array([100, 50, 50, 10])

q, dq, ddq = pg.generate_trajectory(q_start, via_points, 0, t_list, n_list)

while 1:
    for i in range(len(q)):
        int = time.time()
        theta = arm.getArmPosition()
        theta_dot = arm.getArmVelocity()
        u = dy.inertia(q[i]) @ (ddq[i] + Kp*(q[i] - theta) + Kd*(dq[i] - theta_dot)) + dy.gravity(theta) + dy.centrifugalterms(theta, theta_dot)
        u = u*0.01
        # arm.setArmTorque(u)
        arm.setArmPosition(q[i]*0.5)
        # print(arm.getArmTorque())
        now = time.time() - int

        time.sleep(0.1 - now)
