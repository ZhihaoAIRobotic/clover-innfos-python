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

def sin_traj(initial_point, t, n):
    # Note: The t and n will dictate how many time points in the trajectory

    traj = []
    init_T = fkin.fk(initial_point)

    # First, incline the cup by 45 degrees to pour the milk
    # Note, record this value, pain in the ass
    # R = np.array([[0, -1, 0],
    #               [-np.cos(np.pi/4), 0, -np.sin(np.pi/4)],
    #               [-np.sin(np.pi/4), 0, np.cos(np.pi/4)]])

    traj.append(init_T)

    # Tilt Done!

    x = init_T[0, 3]
    y = init_T[1, 3]
    z = init_T[2, 3]
    init_x = x

    tp = t / n

    sinx = []
    siny = []

    # x = 0
    # y = 0

    for i in range(int(3 * n / 4)):
        x = init_x + 20 * 0.05 * math.exp(-15 * y) * np.sin((50.84 * 2 * np.pi * y) + np.pi/2)
        # x = init_x + 0.05 * np.sin(55 * 2 * np.pi * y)
        y = y + 0.05 / (3* n / 4)
        # x = 750000 * math.exp(-100 * y) * np.sin(100 * 2 * np.pi * y)

        sinx.append(x)
        siny.append(y)

    for i in range(int(n / 4)):
        x = x
        y = y - 0.05 / (n / 4)

        sinx.append(x)
        siny.append(y)

    q_list = []
    q_list.append(initial_point)
    q_list.append(kin.ik(init_T, np.array([0, *initial_point]), orientation="all")[1:])

    px_list = []
    py_list = []


    i = 0
    for i in range(len(sinx)):
        p = np.array([sinx[i], siny[i], z])
        T = np.array([[*init_T[0, :3], sinx[i]],
                      [*init_T[1, :3], siny[i]],
                      [*init_T[2, :3], z],
                      [0, 0, 0, 1]])

        guess = [0, *q_list[-1]]
        q = kin.ik(T, guess, orientation="all")
        q = q[1:]

        q_list.append(q)
        px_list.append(p[0])
        py_list.append(p[1])
        traj.append(T)

    traj = np.array(traj)

    theta = np.linspace(np.pi/4, np.pi/2, int(n)+1)

    for i in range(len(theta+1)):

        rot = np.array([[0, -1, 0],
                        [-np.sin(theta[i]), 0, np.cos(theta[i])],
                        [-np.cos(theta[i]), 0, -np.sin(theta[i])]])

        traj[i][:3, :3] = rot


    return np.array(q_list), traj, np.array(px_list), np.array(py_list)

def dq_to_w(q, dq):
    w = np.zeros([len(dq), 4])
    for i in range(len(dq)):
        w[i] = 2 * dq * np.linalg.inv(q[i])
    return w

def ddq_to_dw(q, dq, ddq):
    dw = np.zeros([len(ddq), 4])
    for i in range(len(ddq)):
        w = dq_to_w(q[i], dq[i])
        dw = (2 * ddq[i] - w * dq[i]) * np.linalg.inv(q[i])
    return dw


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    q_list, t_list, x, y= sin_traj(np.array([0, 0, -1.570796, -0.785398, 1.570796, 0]), 1, 100)

    pose, q = T_to_pose(t_list)

    print(t_list)


    # w = dq_to_w(q, q_list)

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


    np.save("coffee_pose_fix", pose)




