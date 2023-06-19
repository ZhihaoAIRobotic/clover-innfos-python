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

def straight_traj(initial_point, l, t, n):

    init_T = fkin.fk(initial_point)

    init_x = init_T[0, 3]

    x_list = []
    y_list = []
    traj_T = []

    for i in range(n):

        x = init_x - l / n * i

        T = np.array([[*init_T[0, 0:3], x],
                      [*init_T[1, :]],
                      [*init_T[2, :]],
                      [0, 0, 0, 1]])

        x_list.append(x)
        y_list.append(init_T[1, 3])
        traj_T.append(T)

    return traj_T, x_list, y_list



if __name__ == "__main__":
    import matplotlib.pyplot as plt
    t_list, x, y = straight_traj(np.array([0.07875, -0.2064, -1.334, 1.55, -1.409, 3.185]), 0.1, 1, 100)

    pose, q = T_to_pose(t_list)

    print(t_list)


    # w = dq_to_w(q, q_list)

    graphlist = np.array(x).reshape(1, np.array(x).shape[0])
    graphlist2 = np.array(y).reshape(1, np.array(y).shape[0])

    for i in range(len(graphlist)):
        # print(row)
        plt.plot(graphlist[i], graphlist2[i])
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("Trajectory of the cup")
    plt.show()

    np.save("buttonpresspose", pose)

