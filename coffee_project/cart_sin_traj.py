import numpy as np
from scps.ikpykinematics import Kinematics
from kinematics import fkin
import math

kin = Kinematics()


def sin_traj(initial_point, t, n):
    # Note: The t and n will dictate how many time points in the trajectory

    traj = []
    init_T = fkin.fk(initial_point)

    # First, incline the cup by 45 degrees to pour the milk
    # Note, record this value, pain in the ass
    # R = np.array([[0, -1, 0],
    #               [-np.cos(np.pi/4), 0, -np.sin(np.pi/4)],
    #               [-np.sin(np.pi/4), 0, np.cos(np.pi/4)]])

    T = np.array([[*R[0, :], init_T[0, 3]],
                  [*R[1, :], init_T[1, 3]],
                  [*R[2, :], init_T[2, 3]],
                  [0, 0, 0, 1]])

    traj.append(T)

    # Tilt Done!

    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    init_x = x

    tp = t / n

    sinx = []
    siny = []

    # x = 0
    # y = 0

    for i in range(int(n / 2)):
        x = init_x + 750000 * math.exp(-100 * y) * np.sin(100 * 2 * np.pi * y)
        y = y + 0.03 / (n / 2)
        # x = 750000 * math.exp(-100 * y) * np.sin(100 * 2 * np.pi * y)

        sinx.append(x)
        siny.append(y)

    for i in range(int(n / 2)):
        x = x
        y = y - 0.03 / (n / 2)

        sinx.append(x)
        siny.append(y)

    q_list = []
    q_list.append(initial_point)
    q_list.append(kin.ik(T, np.array([0, *initial_point]), orientation="all")[1:])



    px_list = []
    py_list = []


    i = 0
    for i in range(len(sinx)):
        p = np.array([sinx[i], siny[i], z])
        T = np.array([[*R[0, :], sinx[i]],
                      [*R[1, :], siny[i]],
                      [*R[2, :], z],
                      [0, 0, 0, 1]])

        guess = [0, *q_list[-1]]
        q = kin.ik(T, guess, orientation="all")
        q = q[1:]

        q_list.append(q)
        px_list.append(p[0])
        py_list.append(p[1])
        traj.append(T)

    return np.array(q_list), np.array(traj), np.array(px_list), np.array(py_list)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    q_list, t_list, x, y = sin_traj(np.array([2.15e-05, 2.15e-05, -1.566, -1.619, 1.672, 2.15e-05]), 1, 100)

    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])
    errorlist = graphlist - graphlist2

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist[i], graphlist2[i])
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the cup")
    plt.show()


