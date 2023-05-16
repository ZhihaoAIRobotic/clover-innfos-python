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
# errorlist = []

q0 = [0, 0, 0, 0, 0, 0]
dq0 = [0, 0, 0, 0, 0, 0]
ddq0 = [0, 0, 0, 0, 0, 0]
q_start = np.array([q0, dq0, ddq0])

via_points = np.array([[0.0757261258, -0.12336681, 0.482877920, 1.57079633, 1.37079633, 2.67079633],
                           [0.07568567, -0.1744927, 0.359509, 1.57079633, 1.07079633, 1.77079633],
                           [0.07572613, -0.166367, 0.49851318, 0.79827042, 1.05037231, 1.93485616]])

t_list = np.array([10.0, 15.0, 20.0])
n_list = np.array([100, 50, 50])

x, dx, ddx = pgc.generate_trajectory(q_start, via_points, 0, t_list, n_list)

while 1:
    for i in range(len(x)):
        int = time.time()
        theta = arm.getArmPosition()
        theta_dot = arm.getArmVelocity()
        j = jacobian.get_jacobian_from_model(theta)
        # y = np.linalg.inv(j) @ (ddx + Kd * (dx - j@theta_dot) + Kp * (x - fkin.fk_pose(theta)) - jacobian.get_dJ(theta, theta_dot) @ theta_dot)
        y = np.linalg.inv(j) @ (ddx + Kd * (dx - j @ theta_dot) + Kp * (x - fkin.fk_pose(theta)) - jacobian.get_eng_dJ(theta, theta_dot) @ theta_dot)
        u = dy.inertia(theta) @ y + dy.gravity(theta) + dy.centrifugalterms(theta, theta_dot)

        arm.setArmTorque(u)

    arm.setArmTorque(np.zeros(6))
