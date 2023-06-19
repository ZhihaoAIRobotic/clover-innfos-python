"""
This is a coffee art making trajectory

There are 3 phases;
1. The Base
    a. Circular motion, and high altitude, to ensure the milk pierces the espresso.
    b. The circular motion is to ensure the milk is mixed with the espresso.
    c. The tilt of the arm is from 45 degrees to 65 degrees

The arm resets to 0 degrees for a while for the coffee arm to tilt to 70 degrees

2. The Art
    a. Constant Velocity
    b. The tilt of the arm is from 65 degrees 85 degrees
    c. Small wave like motions

The arm tilts upwards to 75 degrees for a bit, then re-adjusts to the bottom of the coffee cup with a 0.05m height
difference, then tilt to 85 degrees

3. The Finish
    a. Constant Velocity
    b. The tilt of the arm is from 85 degrees to 95 degrees
    c. Straight line cutting through to the other side of the cup


Author: Sim Heng Yi
Company: Clover Labs CUHK
"""

import numpy as np
import matplotlib.pyplot as plt
from scps.ikpykinematics import Kinematics
from kinematics import fkin
import math
from scipy.spatial.transform import Rotation as R
from mpl_toolkits import mplot3d

kin = Kinematics()
pi = np.pi

def T_to_pose(T):
    p = np.zeros([len(T), 3])
    q = np.zeros([len(T), 4])
    for i in range(len(T)):
        p[i] = T[i][:3, 3]
        rot = R.from_matrix(T[i][:3, :3])
        q[i] = rot.as_quat()

    pose = np.concatenate((p, q), axis=1)

    return pose, q

"""
Design the circle motion in the base trajectory

Params:
    initial_point: The initial point of the arm (tilted at 45 degrees)
    t: The time taken for the arm to complete the circle
"""
def base(initial_point, t):

    n = t/0.01

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []


    return T_list, x_list, y_list, z_list

"""
This stage will readjust the arm to 0 degrees, and then tilt to 65 degrees

Params:
    initial point = The last point of the base trajectory
    t: The time taken for the arm to complete the circle
"""

def readjust(initial_point, t):
    n = t/0.01

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []


    return T_list, x_list, y_list, z_list

"""
Design stage will readjust generate the art design in the coffee while tilting the arm from 65 degrees to 85 degrees.

Params:
    initial point = The last point of the readjust trajectory
    t: The time taken for the arm to complete the circle
"""

def design(initial_point, t):
    n = t / 0.01

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []

    step_size1 = (pi / 16) / (n / 5)
    y_step1 = np.arange(0, pi / 16, step_size1)

    for i in range(len(y_step1)):
        x = init_x + np.sin((pi / 2 * y_step1[i]) + 0.9) - 1.69
        y = init_y + y_step1[i]

        x_list.append(x)
        y_list.append(y)

    step_size2 = (pi/8) / (n / 5)
    y_step2 = np.arange(-pi / 16, pi / 16, step_size2)

    for i in range(len(y_step2)):
        x = init_x + 1.2 * np.sin((pi/2 * y_step2[i]) + pi/2) - 1.9
        y = init_y + y_step2[i]

        x_list.append(x)
        y_list.append(y)

    step_size3 = (pi / 10 + pi / 16) / (n / 5)
    y_step3 = np.arange(pi / 16, -pi / 10, -step_size3)

    for i in range(len(y_step3)):
        x = init_x + np.sin((pi/2 * y_step3[i]) + 8.1) - 1.6
        y = init_y + y_step3[i]

        x_list.append(x)
        y_list.append(y)

    step_size4 = (pi / 10 + pi / 8) / (n / 5)
    y_step3 = np.arange(-pi / 10, pi / 8, step_size3)

    for i in range(len(y_step3)):
        x = init_x + 0.8 * np.sin((pi / 2 * y_step3[i]) + 1.4) - 1.25
        y = init_y + y_step3[i]

        x_list.append(x)
        y_list.append(y)



    return T_list, x_list, y_list, z_list

"""
This stage will readjust the arm to 75 degrees, move upwards and to the base of the cup, then tilt to 85 degrees

Params:
    initial point = The last point of the design trajectory
    t: The time taken for the arm to complete the circle
"""

def readjust2(initial_point, t):
    n = t / 0.01

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []

    return T_list, x_list, y_list, z_list

"""
The finish simply moves to the other side of the cup, and tilt to 95 degrees from 85 degrees

Params:
    initial point = The last point of the readjust2 trajectory
    t: The time taken for the arm to complete the circle
"""

def finish(initial_point, t):
    n = t / 0.01

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []

    return T_list, x_list, y_list, z_list


if __name__ == "__main__":

    T, x, y, z = design(np.array([0, 0, -1.570796, -0.785398, 1.570796, 0]), 10)

    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist2[i], graphlist[i])
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the cup")
    plt.show()
