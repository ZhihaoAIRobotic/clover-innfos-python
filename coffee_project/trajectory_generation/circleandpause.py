import numpy as np
from scps.ikpykinematics import Kinematics
from kinematics import fkin
import math
from scipy.spatial.transform import Rotation as R

kin = Kinematics()

def T_to_pose(T):
    p = np.zeros([len(T), 3])
    q = np.zeros([len(T), 4])
    for i in range(len(T)):
        p[i] = T[i][:3, 3]
        rot = R.from_matrix(T[i][:3, :3])
        q[i] = rot.as_quat()

    pose = np.concatenate((p, q), axis=1)

    return pose, q

def circle(initial_point, r, n):

    init_T = kin.fk(initial_point)

    init_x = init_T[0, 3]
    init_y = init_T[1, 3]

    x_list = []
    y_list = []
    T_list = np.zeros([n, 4, 4])
    x_list.append(init_x)
    y_list.append(init_y)

    # Move forward
    for i in range(int(n / 8 )):
        y = np.array(y_list)[-1] + 0.025 / (n/8)

        y_list.append(y)

        init_T[1, 3] = y

        T_list[i] = init_T

    for i in range(int(n/8), int(3 * n/8)):
        x = np.array(x_list)[-1] + 0.05 / (n/4)

        x_list.append(x)

        T = T_list[i-1]
        T[0, 3] = x

        T_list[i] = T

    for i in range(int(3 * n/8), int(5 * n/8)):

        y = np.array(y_list)[-1] - 0.05 / (n/4)

        y_list.append(y)

        T = T_list[i-1]
        T[1, 3] = y

        T_list[i] = T

    for i in range(int(5 * n / 8), int(7 * n / 8)):
        x = np.array(x_list)[-1] - 0.05 / (n / 4)

        x_list.append(x)

        T = T_list[i-1]
        T[0, 3] = x

        T_list[i] = T

    for i in range(int(7 * n / 8), int(n)):
        y = np.array(y_list)[-1] + 0.025 / (n / 8)

        y_list.append(y)

        T = T_list[i-1]
        T[1, 3] = y

        T_list[i] = T

    x_l = np.zeros(len(T_list))
    y_l = np.zeros(len(T_list))

    for i in range(len(T_list)):
        trans = T_list[i]

        x_l[i] = trans[0, 3]
        y_l[i] = trans[1, 3]

    return T_list, x_l, y_l

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    t_list, x, y = circle(np.array([0, 0, -1.570796, -0.785398, 1.570796, 0]), 1, 80)

    print(y)

    pose, q = T_to_pose(t_list)
    #
    # print(t_list)


    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])
    errorlist = graphlist - graphlist2

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist2[i], graphlist[i])
        plt.ylabel("x (meters)")
        plt.xlabel("y (meters)")
        plt.title("Trajectory of the cup")
    plt.show()


    np.save("milk_froth_circle", pose)


