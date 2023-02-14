#!/bin/env python3
# encoding: utf-8
"""
# Rigid Body kinematics illustrated in code.

The following is a simple illustration of serial chain rigid body dynamics using
Denavit Hartenburg parameterization (original DH, not modified DH).

>NOTE: The code uses a couple *advanced* python techniques to help with readability.
>
> These include [expression list unpacking](https://docs.python.org/3/reference/expressions.html#expression-lists)
> and [target list assignment](https://docs.python.org/3/reference/simple_stmts.html#assignment-statements)

Authors: Traveler Hauptman,
"""

import numpy as np
from copy import copy, deepcopy
from math import atan2, sqrt, pow, sin, cos
import matplotlib.pyplot as plt
import sys

pi = np.pi
pi_2 = pi / 2


class DH(object):
    """
        A Denavit Hartenburg parameterization of a rigid body transform.

        Instantiate: `j1 = DH(0, pi_2, 0.1, 0)`
        Use:         `j1(pi)`

        Instantiate prismatic joint: `j2 = DH(1, 0, 0, pi, prismatic = True)`
    """
    precision = 15  # Nicer printing of orthogonal transforms at slight cost of precision

    def __init__(self, d, θ, a, α, prismatic=False):
        # selector will be used to apply the joint state to rotation or prismatic motion.
        self.selector = (1, 0) if not prismatic else (0, 1)
        self.parameters = (d, θ, a, α)  # To be unpacked for clarity later

    def calc(self, q):
        """ Given joint state parameter, return a transform to the next frame """
        d, θ, a, α = self.parameters
        R, P = self.selector

        # Create shorthand for clarity. Combine joint state with selector and add to θ
        cθ = np.cos(θ + R * q)
        sθ = np.sin(θ + R * q)
        cα = np.cos(α)
        sα = np.sin(α)

        d = d + P * q  # Apply joint state to prismatic parameter

        # DH Transform matrix
        T = [
            [cθ, -sθ * cα, sθ * sα, a * cθ],
            [sθ, cθ * cα, -cθ * sα, a * sθ],
            [0, sα, cα, d],
            [0, 0, 0, 1],
        ]
        return np.around(np.array(T), self.precision)


"""
    A rigid body serial chain is a sequence of transforms, each with a single 
    state variable.

    We capture this as a list of transform objects, each of which returns a
    homogeneous transform matrix when called with the current state.
"""


def fk(chain, q):

    Te = np.identity(4)
    for tf, qi in zip(chain, q):
        Te = Te @ tf.calc(qi)
    return Te  # Transform of end effector


def jacobian(chain, q):
    frame_0 = np.identity(4)
    Os = [frame_0]  # List of Origins, starting with the origin
    for tf, qi in zip(chain, q):
        Os.append(Os[-1] @ tf.calc(qi))

    Te = Os[-1]  # End effector transform
    Jcolumns = []
    for O in Os[:-1]:
        z = O[0:3, 2]  # 3rd column (z cosine vector)
        p = Te[0:3, 3] - O[0:3, 3]  # 4th column (position)
        # Build j as list of columns, to be transposed at the end
        Jcolumns.append(np.hstack([np.cross(z, p), z]))

    return np.array(Jcolumns).T


'''
    This Inverse Kinematics Algorithm allows for different methods of calculating the orientation error.
    The methods include: Euler_Angles, Angle_and_Axis and Quaternion.

    The input parameters are: The DH chain, initial guess, desired transformation matrix, number of iterations, damping and method
    The output is: The joint angles that will result in the desired transformation matrix
'''


def ik(chain, initial_guess, Transform, iterations, k=0.02, method="Euler_Angles"):
    initial_transform = fk(chain, initial_guess)
    q = np.array(initial_guess)

    # Method Selection
    if method == "Euler_Angles":
        def eomg(T):
            # Calculate the euler angles from the rotation matrix
            ee_omg = np.zeros(3)
            ee_omg[0] = atan2(T[2, 1], T[2, 2])
            ee_omg[1] = atan2(-T[2, 0], sqrt(pow(T[2, 1], 2) + pow(T[2, 2], 2)))
            ee_omg[2] = atan2(T[1, 0], T[0, 0])

            T_omg = np.zeros(3)
            T_omg[0] = atan2(Transform[2, 1], Transform[2, 2])
            T_omg[1] = atan2(-Transform[2, 0], sqrt(pow(Transform[2, 1], 2) + pow(Transform[2, 2], 2)))
            T_omg[2] = atan2(Transform[1, 0], Transform[0, 0])

            # Calculate the orientation error
            e_omg = T_omg - ee_omg

            return e_omg

    if method == "Angle_and_Axis":
        def eomg(T):
            # Calculate the orientation error
            e_omg = 1 / 2 * (np.cross(T[0:3, 0], Transform[0:3, 0]) + np.cross(T[0:3, 1], Transform[0:3, 1]) + np.cross(
                T[0:3, 2], Transform[0:3, 2]))

            return e_omg

    if method == "Quaternion":
        def Quat(T):

            # Calculate the quaternion from the rotation matrix
            n = 1 / 2 * sqrt(1 + T[0, 0] + T[1, 1] + T[2, 2])

            if Transform[0, 3] > 0:
                sign = 1
            else:
                sign = -1

            e = np.zeros(3)
            e[0] = sign * (T[2, 1] - T[1, 2]) * 1 / 2 * sqrt(T[0, 0] - T[1, 1] - T[2, 2] + 1)
            e[1] = sign * (T[0, 2] - T[2, 0]) * 1 / 2 * sqrt(T[1, 1] - T[2, 2] - T[1, 1] + 1)
            e[2] = sign * (T[1, 0] - T[0, 1]) * 1 / 2 * sqrt(T[2, 2] - T[0, 0] - T[1, 1] + 1)

            return n, e

        def eomg(T):

            # for desired Transform
            n_d = Quat(Transform)[0]
            e_d = Quat(Transform)[1]

            # for current Transform
            n_e = Quat(T)[0]
            e_e = Quat(T)[1]

            # Calculate the skew-symmetric of e_d
            skew_ed = np.array([[0, -e_d[2], e_d[1]],
                                [e_d[2], 0, -e_d[0]],
                                [-e_d[1], e_d[0], 0]])

            # Calculate the orientation error
            e_omg = n_e * e_d - n_d * e_e - skew_ed @ e_e

            return e_omg

    # Calculate the error
    pos_error = Transform[0:3, 3] - initial_transform[0:3, 3]
    omg_error = eomg(initial_transform)
    # print(omg_error)

    error = np.array([*pos_error, *omg_error]).reshape(6, 1)

    # Build the error condition
    condition = np.linalg.norm(pos_error) > 0.01 \
                or np.linalg.norm(omg_error) > 0.00005

    # initialize the graphing variable library
    graphlist = []

    i = 0

    while condition and i < iterations:

        # To reset the joint values once the loop has restarted
        if i == 0:
            q = np.array(initial_guess)

        # # Randomize the joint values once the pseudo-inverse jacobian exceeds a threshold
        # if np.linalg.cond(np.linalg.pinv(jacobian(chain, q))) < 1000000:
        #     pass
        # else:
        #     tolerance = (np.random.rand( 6 ) - 0.5) * 1
        #     q = q + tolerance

        # Calculate the pseudo inverse of the geometric jacobian

        pseudoinv = np.linalg.pinv(jacobian(chain, q))

        # Calculate the error with gain
        gain_error = np.dot(error, k)

        # Calculate the derivative of the joint values
        dq = np.dot(pseudoinv, gain_error).reshape(1, 6)

        # Iterate
        q = q \
            + dq

        q = np.array(q).reshape(-1)

        q_transform = fk(chain, q)

        # Calculate the error for the next iteration
        pos_error = Transform[0:3, 3] - q_transform[0:3, 3]
        omg_error = eomg(q_transform)
        error = [*pos_error, *omg_error]

        # Build the error condition
        condition = np.linalg.norm(pos_error) > 0.01 \
                    or np.linalg.norm(omg_error) > 0.00005

        # Build the graphing variable library
        graphlist.append(q)

        # Define the limits in terms of Radians

        l1 = ((7 * pi) / 9)
        l2 = pi_2
        l3 = 2 * pi

        # Set the joint limits

        if q[0] < -l1:
            q[0] = 0

        if q[0] > l1:
            q[0] = 0

        if q[1] < -l2:
            q[1] = 0

        if q[1] > l2:
            q[1] = 0

        if q[2] < -l1:
            q[2] = 0

        if q[2] > l1:
            q[2] = 0

        if q[3] < -l1:
            q[3] = 0

        if q[3] > l1:
            q[3] = 0

        if q[4] < -l1:
            q[4] = 0

        if q[4] > l1:
            q[4] = 0

        if q[5] < -l3:
            q[5] = 0

        if q[5] > l3:
            q[5] = 0

        # Restart the loop if IK algorithm does not converge
        # if condition == True:
        #
        #     i = 0

        i = i + 1

    # Normalize joint values that exceed pi
    for j in range(len(q)):
        if q[j] > 0:
            for i in range(int(abs(((q[j] / ((2 * np.pi))) + 1)))):
                if q[j] > ((2 * np.pi)):
                    q[j] = q[j] - ((2 * np.pi))
                if q[j] > np.pi:
                    q[j] = (-(2 * np.pi)) + q[j]

        elif q[j] < 0:
            for i in range(int(((abs(q[j]) / ((2 * np.pi))) + 1))):
                if q[j] < -(2 * np.pi):
                    q[j] = q[j] + (2 * np.pi)
                if -(2 * np.pi) < q[j] < -np.pi:
                    q[j] = (2 * np.pi) + q[j]

    return (q, not condition, graphlist, dq)


################# np.array pretty print #################################
def array_clean_print(sep=' ', vert='|', pad=10, precision=4):
    def prettyprint(a):
        s = "\n"
        if len(a.shape) == 1:
            a = [a]  # Make array appear 2D if it's 1D
        for row in a:
            s += vert + sep + sep.join([f"{x: ^ {pad}.{precision}g}" for x in row]) + vert + "\n"
        return s

    return prettyprint


np.set_string_function(array_clean_print(), repr=False)
np.set_string_function(array_clean_print(), repr=True)

if __name__ == "__main__":

    # DH_parameters[i] => Rigid body transform from Frame(i-1) on Axis(i) to Frame(i) on Axis(i+1)
    DH_parameters = {
        "": ['d', 'θ', 'a', 'α'],
        1: [0.105, pi_2, 0.0, pi_2],
        2: [0.0, pi_2, 0.17447, 0.0],
        3: [0.0, 0.0, 0.17442, pi],
        4: [-0.080113, pi_2, 0.0, pi_2],
        5: [0.080089, 0.0, 0.0, pi_2],
        6: [0.0, 0.0, 0.0, 0.0],
    }

    robot_chain = [DH(*DH_parameters[i]) for i in range(1, 7)]

    forward_kinematics = fk(robot_chain, [0, 1, 0, 0, 0, 0])
    print("Forward kinematics", forward_kinematics)

    j = jacobian(robot_chain, [0, 0, 0, 0, 0, 0])
    # print("Jacobian: J(q) | q=0,0,0,0,0,0", j)
    # print("Jacobian: J(q)*dq | dq=1,0,0,0,0,0", j @ [1, 0, 0, 0, 0, 0])

    # The inverse kinematics orientation error method parameters consists of: Euler_Angles, Angle_and_Axis, Quaternion

    Inverse_kinematics, condition, thetalist, q_vel = ik(robot_chain, [0, 0, 0, 0, 0, 0], forward_kinematics, 500, 0.5, method="Quaternion")
    print("Inverse kinematics", Inverse_kinematics, condition)
    print("Joint Velocity", q_vel)
    # print([Inverse_kinematics[0], Inverse_kinematics[1], Inverse_kinematics[2], Inverse_kinematics[3], Inverse_kinematics[4], Inverse_kinematics[5]])

    # print("")
    # rad_to_degree = Inverse_kinematics[:]*(180/pi)

    # print(rad_to_degree)

    # Check if the Transform of the Inverse kinematics is the same as the Forward kinematics
    Solution_Check = fk(robot_chain, Inverse_kinematics)
    print("Solution Check", Solution_Check)

    graphlist = np.array(thetalist).reshape(6, np.array(thetalist).shape[0])

    for row in graphlist:
        plt.plot(row)
        # plt.ylabel("θ (°)")
        plt.ylabel("θ (radians)")
        plt.xlabel("Iterations")
        plt.title("Joint iterations")
    plt.show()
