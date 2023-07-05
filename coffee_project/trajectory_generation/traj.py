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
Design the circle motion in the base trajectory, and tilt to 60 degrees

Params:
    initial_point: The initial point of the arm, Center of the circle, No tilt
    t: The time taken for the arm to complete the circle
"""

def base(initial_point, t):
    init_T = fkin.fk(initial_point)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    y_os = init_y + 0.175
    z_os = init_z + 0.03315

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []
    T_list.append(init_T)

    r = 0.025
    n = t / 0.01

    # Move to the initial point of circle

    cir_x = init_x + r

    step = (cir_x - init_x) / (n/4)
    x_step = np.arange(init_x, cir_x, step)
    theta = np.linspace(0, pi / 4, len(x_step))

    for i in range(len(x_step)):
        x = x_step[i]
        y = y_os

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        z_ee = z_os + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), y_ee],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), z_ee],
                      [0, 0, 0, 1]])

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)
        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)
        T_list.append(T)

    # The circle motion

    theta_step = 2 * pi / (3 * n / 4)
    theta = np.arange(0, 2 * np.pi, theta_step)
    theta2 = np.linspace(pi / 4, pi / 3, len(theta))

    for i in range(len(theta)):
        x = r * math.cos(theta[i]) + init_x
        y = r * math.sin(theta[i]) + y_os

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta2[i]) + 0.03315 * np.sin(theta2[i]))
        z_ee = z_os + 0.175 * np.sin(theta2[i]) - 0.03315 * np.cos(theta2[i])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta2[i]), 0, np.cos(theta2[i]), y_ee],
                      [-np.cos(theta2[i]), 0, -np.sin(theta2[i]), z_ee],
                      [0, 0, 0, 1]])
        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)
        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)
        T_list.append(T)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

def readjust(initial_point, final_point, t):
    """
    This stage will readjust the arm to 0 degrees, and then tilt to 60 degrees

    Params:
        initial point = The last point of the base trajectory, in terms of a transformation matrix
        final point = The first point of the design trajectory, moving downwards
        t: The time taken for the arm to complete the circle
    """

    n = t / 0.01

    init_T = initial_point
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    y_os = init_y + 0.175 * np.cos(pi / 3) + 0.03315 * np.sin(pi / 3)
    z_os = init_z - (0.175 * np.sin(pi / 3) - 0.03315 * np.cos(pi / 3))

    x_list = []
    x_list.append(init_x)
    y_list = []
    y_list.append(y_os)
    z_list = []
    z_list.append(z_os)

    T_list = []
    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    # Tilt upwards and move downwards

    theta_step = (pi / 3) / (n / 2)
    theta = np.arange(pi / 3, 0, -theta_step)


    for i in range(int(n / 2)):
        y = y_os
        z = z_os

        x_ee = init_x
        y_ee = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        z_ee = z + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])
        
        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), y_ee],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), z_ee],
                      [0, 0, 0, 1]])

        T_list.append(T)
        x_list.append(init_x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    # Tilt downwards and move to first point of design trajectory
    f_x = final_point[0, 3]
    f_y = final_point[1, 3]
    f_z = final_point[2, 3]

    theta_step = (pi / 3) / (n / 2)
    theta2 = np.arange(0, pi / 3, theta_step)

    x_step = (f_x - ee_x_list[-1]) / (n / 2)
    x_s = np.arange(ee_x_list[-1], f_x, x_step)
    y_step = (f_y - ee_y_list[-1]) / (n / 2)
    y_s = np.arange(ee_y_list[-1], f_y, y_step)
    z_step = (f_z - ee_z_list[-1]) / (n / 2)
    z_s = np.arange(ee_z_list[-1], f_z, z_step)

    for i in range(int(n / 2)):

        x_ee = x_s[i]
        y_ee = y_s[i]
        z_ee = z_s[i]

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta2[i]), 0, np.cos(theta2[i]), y_ee],
                      [-np.cos(theta2[i]), 0, -np.sin(theta2[i]), z_ee],
                      [0, 0, 0, 1]])

        T_list.append(T)

        x = x_ee
        y = y_ee + (0.175 * np.cos(theta2[i]) + 0.03315 * np.sin(theta2[i]))
        z = z_ee - (0.175 * np.sin(theta2[i]) - 0.03315 * np.cos(theta2[i]))

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

"""
Design stage will readjust generate the art design in the coffee while tilting the arm from 60 degrees to 80 degrees.

Params:
    initial point = The last point of the readjust trajectory
    t: The time taken for the arm to complete the circle
"""

def design(initial_point, t):
    n = t / 0.01

    init_T = fkin.fk(initial_point)

    init_T[2, 3] = init_T[2, 3] + 0.15

    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    y_os = init_y + 0.175 * np.cos(np.pi / 3) + 0.03315 * np.sin(np.pi / 3)
    z_os = init_z - (0.175 * np.sin(np.pi / 3) - 0.03315 * np.cos(np.pi / 3))

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    theta_step = ((4 * pi) / 9 - pi / 3) / (n + 3)
    theta = np.arange(pi / 3, 4 * pi / 9, theta_step)
    step_size6 = (pi / 250 + pi / 250) / (n / 6)
    x_step6 = np.arange(pi / 190, -pi / 190, -step_size6)

    for i in range(len(x_step6)):
        y = y_os + 0.8 * np.sin((9 * pi / 4 * x_step6[i]) + 1.57) - 0.816
        x = init_x + x_step6[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        z_ee = z_os + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), y_ee],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    step_size5 = (pi / 250 + pi / 300) / (n / 6)
    y_step5 = np.arange(-pi / 250, pi / 300, step_size5)

    for i in range(len(y_step5)):
        y = y_os + 0.7 * np.sin((8 * pi / 4 * y_step5[i]) + 1.62) - 0.7189
        x = init_x + y_step5[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i + int(n/6)]) + 0.03315 * np.sin(theta[i + int(n/6)]))
        z_ee = z_os + 0.175 * np.sin(theta[i + int(n/6)]) - 0.03315 * np.cos(theta[i + int(n/6)])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i + int(n/6)]), 0, np.cos(theta[i + int(n/6)]), y_ee],
                      [-np.cos(theta[i + int(n/6)]), 0, -np.sin(theta[i + int(n/6)]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    step_size4 = (pi / 300 + pi / 350) / (n / 6)
    y_step4 = np.arange(pi / 300, -pi / 350, -step_size4)

    for i in range(len(y_step4)):
        y = y_os + 0.8 * np.sin((7 * pi / 4 * y_step4[i]) + 1.53) - 0.82341
        x = init_x + y_step4[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i + int(n/3)]) + 0.03315 * np.sin(theta[i + int(n/3)]))
        z_ee = z_os + 0.175 * np.sin(theta[i + int(n/3)]) - 0.03315 * np.cos(theta[i + int(n/3)])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i + int(n/3)]), 0, np.cos(theta[i + int(n/3)]), y_ee],
                      [-np.cos(theta[i + int(n/3)]), 0, -np.sin(theta[i + int(n/3)]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    step_size3 = (pi / 350 + pi / 400) / (n / 6)
    y_step3 = np.arange(-pi / 350, pi / 400, step_size3)

    for i in range(len(y_step3)):
        y = y_os + np.sin((8 * pi / 4 * y_step3[i]) + 7.89) - 1.02645
        x = init_x + y_step3[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i + int(n/2)]) + 0.03315 * np.sin(theta[i + int(n/2)]))
        z_ee = z_os + 0.175 * np.sin(theta[i + int(n/2)]) - 0.03315 * np.cos(theta[i + int(n/2)])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i + int(n/2)]), 0, np.cos(theta[i + int(n/2)]), y_ee],
                      [-np.cos(theta[i + int(n/2)]), 0, -np.sin(theta[i + int(n/2)]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    step_size2 = (pi / 400 + pi / 500) / (n / 6)
    y_step2 = np.arange(pi / 400, -pi / 500, -step_size2)

    for i in range(len(y_step2)):
        y = y_os + 1.2 * np.sin((6 * pi / 4 * y_step2[i]) + pi / 2) - 1.22926
        x = init_x + y_step2[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i + int((2*n)/3)]) + 0.03315 * np.sin(theta[i + int((2*n)/3)]))
        z_ee = z_os + 0.175 * np.sin(theta[i + int((2*n)/3)]) - 0.03315 * np.cos(theta[i + int((2*n)/3)])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i + int((2*n)/3)]), 0, np.cos(theta[i + int((2*n)/3)]), y_ee],
                      [-np.cos(theta[i + int((2*n)/3)]), 0, -np.sin(theta[i + int((2*n)/3)]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    step_size1 = (pi / 500) / (n / 6)
    y_step1 = np.arange(-pi / 500, 0, step_size1)

    for i in range(len(y_step1)):
        y = y_os + np.sin((8 * pi / 4 * y_step1[i]) + 1.7) - 1.02576
        x = init_x + y_step1[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i + int((5*n)/6)]) + 0.03315 * np.sin(theta[i + int((5*n)/6)]))
        z_ee = z_os + 0.175 * np.sin(theta[i + int((5*n)/6)]) - 0.03315 * np.cos(theta[i + int((5*n)/6)])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i + int((5*n)/6)]), 0, np.cos(theta[i + int((5*n)/6)]), y_ee],
                      [-np.cos(theta[i + int((5*n)/6)]), 0, -np.sin(theta[i + int((5*n)/6)]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z_os)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

"""
This stage will readjust the arm to 70 degrees, move upwards and to the base of the cup, then tilt to 80 degrees

Params:
    initial point = The last point of the design trajectory
    t: The time taken for the arm to complete the circle
"""

def readjust2(initial_point, t):
    n = t / 0.01

    init_T = initial_point
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    y_os = init_y + 0.175 * np.cos(4 * np.pi / 9) + 0.03315 * np.sin(4 * np.pi / 9)
    z_os = init_z - (0.175 * np.sin(4 * np.pi / 9) - 0.03315 * np.cos(4 * np.pi / 9))

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    # Move upwards while tilting up
    l = 0.05
    z_s = l / (n/2)
    z_step = np.arange(z_os, z_os + l, z_s)

    theta_step = (4 * (pi/9) - pi/3)/ (n/2)
    theta = np.arange(4 * (pi/9), pi/3, -theta_step)

    for i in range(int(n/2)):
        x = init_x
        y = y_os
        z = z_step[i]

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        z_ee = z + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), y_ee],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    # Move to finish start point

    # z_step2 = np.arange(z_os + l, z_os, -z_s)
    theta_step = (4 * (pi/9) - pi / 3) / (n/2)
    theta = np.arange(pi/3, 4 * (pi/9), theta_step)

    for i in range(int(n/2)):
        x = init_x
        y = y_os
        z = z_os + l

        x_ee = x
        y_ee = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        z_ee = z + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])

        T = np.array([[0, -1, -0, x_ee],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), y_ee],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), z_ee],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(x_ee)
        ee_y_list.append(y_ee)
        ee_z_list.append(z_ee)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

"""
The finish simply moves to the other side of the cup, and tilt to 90 degrees from 80 degrees

Params:
    initial point = The last point of the readjust2 trajectory
    t: The time taken for the arm to complete the circle
"""

def finish(initial_point, t):
    n = t / 0.01

    init_T = initial_point
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    y_os = init_y + 0.175 * np.cos(4 * np.pi / 9) + 0.03315 * np.sin(4 * np.pi / 9)
    z_os = init_z - (0.175 * np.sin(4 * np.pi / 9) - 0.03315 * np.cos(4 * np.pi / 9))

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    # Move to the other side of the cup
    l = 0.045
    y_s = l/n
    y_step = np.arange(y_os, y_os + l, y_s)
    theta_step = (pi / 2 - 4 * (pi/9))/n
    theta = np.arange(4 * (pi/9), pi/2, theta_step)

    for i in range(int(n)):
        x = init_x
        y = y_step[i]
        z = z_os

        ee_x = x
        ee_y = y - (0.175 * np.cos(theta[i]) + 0.03315 * np.sin(theta[i]))
        ee_z = z + 0.175 * np.sin(theta[i]) - 0.03315 * np.cos(theta[i])

        T = np.array([[0, -1, -0, ee_x],
                      [-np.sin(theta[i]), 0, np.cos(theta[i]), ee_y],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), ee_z],
                      [0, 0, 0, 1]])
        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(ee_x)
        ee_y_list.append(ee_y)
        ee_z_list.append(ee_z)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

def cof_to_init(initial, t):

    n = t / 0.01

    init_T = kin.fk(initial)
    print(init_T)
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    y_s = (-0.2837437 + init_y)/n
    z_s = (0.16118173 - init_z)/n

    y_step = np.arange(init_y, - 0.2837437, y_s)
    z_step = np.arange(init_z, 0.16118173, z_s)

    # move to init
    for i in range(len(y_step)-1):

        x = init_x
        y = y_step[i] - 0.12525
        z = z_step[i] + 0.05

        # print(i)
        T = np.array([[0, -1, 0, init_x],
                      [0, 0, 1, y_step[i]],
                      [-1, 0, 0, z_step[i]],
                      [0, 0, 0, 1]])

        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(init_x)
        ee_y_list.append(y_step[i])
        ee_z_list.append(z_step[i])

        print(T)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list
def cof_readjust(initial, t):

    n = t / 0.01

    init_T = initial
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    x = init_x
    y_os = init_y - 0.12525
    z_os = init_z + 0.05

    # Tilt without moving

    theta_step = (pi/4)/n
    theta = np.arange(0, pi/4, theta_step)

    for i in range(len(theta)):
        x = init_x
        y = y_os
        z = z_os

        ee_x = x
        ee_y = y + (0.12525 * np.cos(theta[i]) + 0.05 * np.sin(theta[i]))
        ee_z = z + 0.12525 * np.sin(theta[i]) - 0.05 * np.cos(theta[i])

        T = np.array([[0, -1, 0, ee_x],
                      [np.sin(theta[i]), 0, -np.cos(theta[i]), ee_y],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), ee_z],
                      [0, 0, 0, 1]])

        print(theta[i])

        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(ee_x)
        ee_y_list.append(ee_y)
        ee_z_list.append(ee_z)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list

def cof_design(initial, t):
    n = t / 0.01

    init_T = initial
    init_x = init_T[0, 3]
    init_y = init_T[1, 3]
    init_z = init_T[2, 3]

    x_list = []
    y_list = []
    z_list = []

    ee_x_list = []
    ee_y_list = []
    ee_z_list = []

    T_list = []

    x = init_x
    y_os = init_y - 0.12525 * np.cos(pi / 4) - 0.05 * np.sin(pi / 4)
    z_os = init_z - 0.12525 * np.sin(pi / 4) + 0.05 * np.cos(pi / 4)

    # Tilt without moving

    theta_step = (pi / 4) / n
    theta = np.arange(pi / 4, 0, -theta_step)

    for i in range(int(n)):
        x = init_x
        y = y_os
        z = z_os

        ee_x = x
        ee_y = y + (0.12525 * np.cos(theta[i]) + 0.05 * np.sin(theta[i]))
        ee_z = z + 0.12525 * np.sin(theta[i]) - 0.05 * np.cos(theta[i])

        T = np.array([[0, -1, 0, ee_x],
                      [np.sin(theta[i]), 0, -np.cos(theta[i]), ee_y],
                      [-np.cos(theta[i]), 0, -np.sin(theta[i]), ee_z],
                      [0, 0, 0, 1]])

        T_list.append(T)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        ee_x_list.append(ee_x)
        ee_y_list.append(ee_y)
        ee_z_list.append(ee_z)

    return T_list, x_list, y_list, z_list, ee_x_list, ee_y_list, ee_z_list


def concatenate_x(x1, x2, x3, x4, x5):
    x1 = np.array(x1).reshape(len(x1), 1)
    x2 = np.array(x2).reshape(len(x2), 1)
    x3 = np.array(x3).reshape(len(x3), 1)
    x4 = np.array(x4).reshape(len(x4), 1)
    x5 = np.array(x5).reshape(len(x5), 1)

    x = np.vstack((np.array(x1), np.array(x2), np.array(x3), np.array(x4), np.array(x5)))
    return x

def concatenate_y(y1, y2, y3, y4, y5):
    y1 = np.array(y1).reshape(len(y1), 1)
    y2 = np.array(y2).reshape(len(y2), 1)
    y3 = np.array(y3).reshape(len(y3), 1)
    y4 = np.array(y4).reshape(len(y4), 1)
    y5 = np.array(y5).reshape(len(y5), 1)
    
    y = np.vstack((np.array(y1), np.array(y2), np.array(y3), np.array(y4), np.array(y5)))
    return y

def concatenate_z(z1, z2, z3, z4, z5):
    z1 = np.array(z1).reshape(len(z1), 1)
    z2 = np.array(z2).reshape(len(z2), 1)
    z3 = np.array(z3).reshape(len(z3), 1)
    z4 = np.array(z4).reshape(len(z4), 1)
    z5 = np.array(z5).reshape(len(z5), 1)
    
    z = np.vstack((np.array(z1), np.array(z2), np.array(z3), np.array(z4), np.array(z5)))
    return z

def concatenate_T(T1, T2, T3, T4, T5):
    T = np.vstack((np.array(T1), np.array(T2), np.array(T3), np.array(T4), np.array(T5)))
    return T


def cof_concatenate_x(x1, x2, x3):
    x1 = np.array(x1).reshape(len(x1), 1)
    x2 = np.array(x2).reshape(len(x2), 1)
    x3 = np.array(x3).reshape(len(x3), 1)

    x = np.vstack((np.array(x1), np.array(x2), np.array(x3)))
    return x


def cof_concatenate_y(y1, y2, y3):
    y1 = np.array(y1).reshape(len(y1), 1)
    y2 = np.array(y2).reshape(len(y2), 1)
    y3 = np.array(y3).reshape(len(y3), 1)

    y = np.vstack((np.array(y1), np.array(y2), np.array(y3)))
    return y


def cof_concatenate_z(z1, z2, z3):
    z1 = np.array(z1).reshape(len(z1), 1)
    z2 = np.array(z2).reshape(len(z2), 1)
    z3 = np.array(z3).reshape(len(z3), 1)

    z = np.vstack((np.array(z1), np.array(z2), np.array(z3)))
    return z


def cof_concatenate_T(T1, T2, T3):
    T = np.vstack((np.array(T1), np.array(T2), np.array(T3)))
    return T


if __name__ == "__main__":

    from mpl_toolkits import mplot3d

    T1, x1, y1, z1, ee_x1, ee_y1, ee_z1 = base(np.array([0, 0, -2.356194, -2.356194, 1.570796, 0]), 5)
    T3, x3, y3, z3, ee_x3, ee_y3, ee_z3 = design(np.array([0, 0, -2.356194, -1.308997, 1.570796, 0]), 5)
    T2, x2, y2, z2, ee_x2, ee_y2, ee_z2 = readjust(np.array(T1[-1]), T3[1], 4)
    T4, x4, y4, z4, ee_x4, ee_y4, ee_z4 = readjust2(np.array(T3[-1]), 3)
    T5, x5, y5, z5, ee_x5, ee_y5, ee_z5 = finish(np.array(T4[-1]), 5)

    CT1, Cx1, Cy1, Cz1, Cee_x1, Cee_y1, Cee_z1 = cof_to_init(np.array([0, 0, 0, 0, 0, 0]), 5)
    CT2, Cx2, Cy2, Cz2, Cee_x2, Cee_y2, Cee_z2 = cof_readjust(CT1[-1], 5)
    print(Cy2[-1])
    CT3, Cx3, Cy3, Cz3, Cee_x3, Cee_y3, Cee_z3 = cof_design(CT2[-1], 5)
    print(Cy3[0])
    #
    # print(T3[1])
    # print(T3[-1])
    # print(y3[-1])

    x = concatenate_x(x1, x2, x3, x4, x5)
    y = concatenate_y(y1, y2, y3, y4, y5)
    z = concatenate_z(z1, z2, z3, z4, z5)

    ee_x = concatenate_x(ee_x1, ee_x2, ee_x3, ee_x4, ee_x5)
    ee_y = concatenate_y(ee_y1, ee_y2, ee_y3, ee_y4, ee_y5)
    ee_z = concatenate_z(ee_z1, ee_z2, ee_z3, ee_z4, ee_z5)

    cx = cof_concatenate_x(Cx1, Cx2, Cx3)
    cy = cof_concatenate_y(Cy1, Cy2, Cy3)
    cz = cof_concatenate_z(Cz1, Cz2, Cz3)

    cee_x = cof_concatenate_x(Cee_x1, Cee_x2, Cee_x3)
    cee_y = cof_concatenate_y(Cee_y1, Cee_y2, Cee_y3)
    cee_z = cof_concatenate_z(Cee_z1, Cee_z2, Cee_z3)

    pose1, q = T_to_pose(T1)
    np.save("base", pose1)

    pose2, q = T_to_pose(T2)
    np.save("readjust1", pose2)

    pose3, q = T_to_pose(T3)
    np.save("design", pose3)

    pose4, q = T_to_pose(T4)
    np.save("readjust2", pose4)

    pose5, q = T_to_pose(T5)
    np.save("finish", pose5)

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])
    graphlist3 = np.array(z).reshape(1, np.array(z).shape[0])
    graphlist4 = np.array(ee_x).reshape(1, np.array(ee_x).shape[0])
    graphlist5 = np.array(ee_y).reshape(1, np.array(ee_y).shape[0])
    graphlist6 = np.array(ee_z).reshape(1, np.array(ee_z).shape[0])

    c_graphlist = np.array(cx).reshape(1, np.array(cx).shape[0])
    c_graphlist2 = np.array(cy).reshape(1, np.array(cy).shape[0])
    c_graphlist3 = np.array(cz).reshape(1, np.array(cz).shape[0])
    # print(c_graphlist)
    # print(c_graphlist2.shape)
    # print(c_graphlist3.shape)

    c_graphlist4 = np.array(cee_x).reshape(1, np.array(cee_x).shape[0])
    c_graphlist5 = np.array(cee_y).reshape(1, np.array(cee_y).shape[0])
    c_graphlist6 = np.array(cee_z).reshape(1, np.array(cee_z).shape[0])

    # 2D plot
    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist[i], graphlist2[i], graphlist3[i], 'red')
        plt.plot(graphlist4[i], graphlist5[i], graphlist6[i], 'blue')
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the pitcher tip")
    plt.show()

    # 3D plot
    # ax.plot3D(graphlist4, graphlist5, graphlist6, 'red')
    # plt.show()

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    for i in range(len(c_graphlist)):
        # print(row)
        plt.plot(c_graphlist[i], c_graphlist2[i], c_graphlist3[i], 'red')
        plt.plot(c_graphlist4[i], c_graphlist5[i], c_graphlist6[i], 'blue')
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the coffee tip")
    plt.show()

    # ax.plot3D(c_graphlist4, c_graphlist5, c_graphlist6, 'red')
    # plt.show()