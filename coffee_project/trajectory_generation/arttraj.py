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
Design the circle motion in the base trajectory, and tilt to 65 degrees

Params:
    initial_point: The initial point of the arm, Center of the circle
    t: The time taken for the arm to complete the circle
"""
def base(initial_point, t):

    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    T_list = []

    r = 0.025

    # Move to the initial point of circle

    cir_x = init_x + r
    n = 0.5 / 0.01

    step = (cir_x - init_x) / n
    x_step = np.arange(init_x, cir_x, step)

    for i in range(len(x_step)):
        x = x_step[i]
        y = init_y

        x_list.append(x)
        y_list.append(y)

    # The circle motion

    n = t / 0.01
    theta_step = 2 * pi / n
    theta = np.arange(0, 2 * np.pi, theta_step)

    for i in range(len(theta)):
        x = r * math.cos(theta[i]) + init_x
        y = r * math.sin(theta[i]) + init_y

        x_list.append(x)
        y_list.append(y)


    # Tilt to 65 degrees


    return T_list, x_list, y_list, z_list

def readjust(initial_point, final_point, t):

    """
    This stage will readjust the arm to 0 degrees, and then tilt to 65 degrees

    Params:
        initial point = The last point of the base trajectory
        final point = The first point of the design trajectory, moving downwards
        t: The time taken for the arm to complete the circle
    """

    n = t/0.01

    init_T = kin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    x_list.append(init_x)
    y_list = []
    y_list.append(init_y)
    z_list = []

    T_list = []

    # Move downwards

    # Tilt upwards

    # Tilt downwards


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
    print(init_T)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]
    x_list = []
    x_list.append(init_x)
    y_list = []
    y_list.append(init_y)
    z_list = []
    T_list = []

    step_size6 = (pi / 75 + pi / 75) / (n / 5)
    x_step6 = np.arange(pi / 75, -pi / 75, -step_size6)

    for i in range(len(x_step6)):
        y = init_y + 0.7 * np.sin((6.5 * pi / 4 * x_step6[i]) + 1.56) - 0.6908
        x = init_x + x_step6[i]

        x_list.append(x)
        y_list.append(y)

    step_size5 = (pi / 75 + pi / 95) / (n / 5)
    y_step5 = np.arange(-pi / 75, pi / 95, step_size5)

    for i in range(len(y_step5)):
        y = init_y + 0.6 * np.sin((6 * pi / 4 * y_step5[i]) + 1.68) - 0.606
        x = init_x + y_step5[i]

        x_list.append(x)
        y_list.append(y)

    step_size4 = (pi / 125 + pi / 95) / (n / 5)
    y_step4 = np.arange(pi / 95, -pi / 125, -step_size4)

    for i in range(len(y_step4)):
        y = init_y + 0.8 * np.sin((6 * pi / 4 * y_step4[i]) + 1.53) - 0.822
        x = init_x + y_step4[i]

        x_list.append(x)
        y_list.append(y)

    step_size3 = (pi / 125 + pi / 140) / (n / 5)
    y_step3 = np.arange(-pi / 125, pi / 140, step_size3)

    for i in range(len(y_step3)):
        y = init_y + np.sin((5 * pi / 4 * y_step3[i]) + 7.95) - 1.0323
        x = init_x + y_step3[i]

        x_list.append(x)
        y_list.append(y)

    step_size2 = (pi / 150 + pi / 140) / (n / 5)
    y_step2 = np.arange(pi / 140, -pi / 150, -step_size2)

    for i in range(len(y_step2)):
        y = init_y + 1.2 * np.sin((5 * pi / 4 * y_step2[i]) + pi / 2) - 1.2445
        x = init_x + y_step2[i]

        x_list.append(x)
        y_list.append(y)


    step_size1 = (pi / 150) / (n / 5)
    y_step1 = np.arange(-pi / 150, 0, step_size1)

    for i in range(len(y_step1)):
        y = init_y + np.sin((6 * pi / 4 * y_step1[i]) + 1.8) - 1.04
        x = init_x + y_step1[i]

        # T =

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

    # Move upwards while tilting up

    # Move to the base of the cup and upwards

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

    # Move to the other side of the cup


    # Tilt to 95 degrees


    return T_list, x_list, y_list, z_list


if __name__ == "__main__":

    T, x, y, z = design(np.array([0, 0, -1.570796, -0.785398, 1.570796, 0]), 10)
    # T, x, y, z = base(np.array([0, 0, -1.570796, -0.785398, 1.570796, 0]), 10)

    print(x)

    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist[i], graphlist2[i])
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the pitcher tip")
    plt.show()
